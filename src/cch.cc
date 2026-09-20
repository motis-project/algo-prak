#include "osr/routing/cch.h"

#include <cstdint>
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <shared_mutex>
#include <tuple>
#include <vector>

#include "osr/elevation_storage.h"
#include "osr/routing/path_reconstruction.h"
#include "osr/routing/profiles/car.h"
#include "osr/types.h"

namespace osr::cch {
namespace {
constexpr auto const kMagic = std::uint64_t{0x4F53525F43434834ULL};
constexpr auto const kNoNode = std::numeric_limits<std::uint32_t>::max();

struct header {
  std::uint64_t magic_{kMagic};
  std::uint64_t state_count_{};
  std::uint64_t edge_count_{};
};

struct edge_ref {
  std::uint32_t low_{};
  std::uint32_t high_{};
};

std::filesystem::path artifact_path(std::filesystem::path const& p) {
  return p / kFileName;
}

std::uint64_t edge_key(std::uint32_t const a, std::uint32_t const b) {
  return (static_cast<std::uint64_t>(a) << 32U) | b;
}

std::vector<std::uint32_t> make_offsets(ways::routing const& r) {
  auto offsets = std::vector<std::uint32_t>(r.node_ways_.size() + 1U);
  for (auto i = 0U; i != r.node_ways_.size(); ++i) {
    utl::verify(
        r.node_ways_[node_idx_t{i}].size() * 2ULL + offsets[i] < kNoNode,
        "CCH state count exceeds 32 bits");
    offsets[i + 1U] =
        offsets[i] +
        static_cast<std::uint32_t>(r.node_ways_[node_idx_t{i}].size()) * 2U;
  }
  return offsets;
}

template <typename P>
std::uint32_t state_id(std::vector<std::uint32_t> const& offsets,
                       typename P::node const n) {
  return offsets[to_idx(n.n_)] + n.way_ * 2U +
         (n.dir_ == direction::kForward ? 0U : 1U);
}

template <typename P, direction Dir, typename Fn>
void for_each_arc(typename P::parameters const& params,
                  ways::routing const& r,
                  std::vector<std::uint32_t> const& offsets,
                  Fn&& fn) {
  for (auto i = 0U; i != r.node_ways_.size(); ++i) {
    P::resolve_all(
        r, node_idx_t{i}, level_t{0.F}, [&](typename P::node const from) {
          P::template adjacent<Dir, false>(
              params, r, from, nullptr, nullptr, nullptr,
              [&](typename P::node const to, cost_t const cost, distance_t,
                  way_idx_t, std::uint16_t, std::uint16_t,
                  elevation_storage::elevation, bool) {
                if (cost != kInfeasible) {
                  fn(state_id<P>(offsets, from), state_id<P>(offsets, to),
                     cost);
                }
              });
        });
  }
}

bool read_header(std::filesystem::path const& p, header& h) {
  auto in = std::ifstream{artifact_path(p), std::ios::binary};
  if (!in) {
    return false;
  }
  in.read(reinterpret_cast<char*>(&h), sizeof(h));
  if (!in || h.magic_ != kMagic || h.state_count_ >= kNoNode ||
      h.edge_count_ >= kNoNode) {
    return false;
  }
  auto ec = std::error_code{};
  return std::filesystem::file_size(artifact_path(p), ec) ==
             sizeof(header) + h.state_count_ * sizeof(std::uint32_t) +
                 h.edge_count_ * sizeof(edge_ref) &&
         !ec;
}

bool read_artifact(std::filesystem::path const& p,
                   header const& h,
                   std::vector<std::uint32_t>& rank,
                   std::vector<edge_ref>& edges) {
  auto in = std::ifstream{artifact_path(p), std::ios::binary};
  if (!in) {
    return false;
  }
  auto file_header = header{};
  in.read(reinterpret_cast<char*>(&file_header), sizeof(file_header));
  if (!in || file_header.magic_ != kMagic ||
      file_header.state_count_ != h.state_count_ ||
      file_header.edge_count_ != h.edge_count_) {
    return false;
  }
  rank.resize(h.state_count_);
  edges.resize(h.edge_count_);
  in.read(reinterpret_cast<char*>(rank.data()),
          static_cast<std::streamsize>(rank.size() * sizeof(rank.front())));
  in.read(reinterpret_cast<char*>(edges.data()),
          static_cast<std::streamsize>(edges.size() * sizeof(edges.front())));
  if (!in) {
    return false;
  }
  auto seen = std::vector<bool>(rank.size());
  for (auto const r : rank) {
    if (r >= rank.size() || seen[r]) {
      return false;
    }
    seen[r] = true;
  }
  auto previous = std::uint64_t{};
  for (auto const& e : edges) {
    auto const key = edge_key(e.low_, e.high_);
    if (e.low_ >= rank.size() || e.high_ >= rank.size() ||
        rank[e.low_] >= rank[e.high_] || key <= previous) {
      return false;
    }
    previous = key;
  }
  return true;
}
}  // namespace

void preprocess(ways::routing const& r, std::filesystem::path const& out) {
  auto const timer = utl::scoped_timer{"CCH preprocessing"};
  auto const offsets = make_offsets(r);
  auto const n_states = offsets.back();

  auto adj = std::vector<std::vector<std::uint32_t>>(n_states);
  auto seen = hash_set<std::uint64_t>{};
  auto add_edge = [&](std::uint32_t const a, std::uint32_t const b) {
    if (a == b) {
      return;
    }
    auto const lo = std::min(a, b);
    auto const hi = std::max(a, b);
    if (seen.emplace(edge_key(lo, hi)).second) {
      adj[a].push_back(b);
      adj[b].push_back(a);
    }
  };

  auto const collect = [&](auto const& p, auto tag) {
    using P = decltype(tag);
    auto const collect_arc = [&](auto const a, auto const b, cost_t) {
      add_edge(a, b);
    };
    for_each_arc<P, direction::kForward>(p, r, offsets, collect_arc);
    for_each_arc<P, direction::kBackward>(p, r, offsets, collect_arc);
  };
  collect(car::parameters{}, car{});
  collect(bus::parameters{}, bus{});

  auto order = std::vector<std::uint32_t>(n_states);
  for (auto i = 0U; i != n_states; ++i) {
    order[i] = i;
  }
  auto importance = std::vector<std::uint32_t>(n_states);
  for (auto n = 0U; n != r.node_ways_.size(); ++n) {
    for (auto s = offsets[n]; s != offsets[n + 1U]; ++s) {
      importance[s] =
          r.node_importance_.empty() ? 0U : r.node_importance_[node_idx_t{n}];
    }
  }
  std::stable_sort(begin(order), end(order), [&](auto const a, auto const b) {
    return std::tuple{importance[a], adj[a].size()} <
           std::tuple{importance[b], adj[b].size()};
  });

  auto rank = std::vector<std::uint32_t>(n_states);
  for (auto i = 0U; i != order.size(); ++i) {
    rank[order[i]] = i;
  }

  for (auto const x : order) {
    auto higher = std::vector<std::uint32_t>{};
    for (auto const nb : adj[x]) {
      if (rank[nb] > rank[x]) {
        higher.push_back(nb);
      }
    }
    for (auto i = 0U; i < higher.size(); ++i) {
      for (auto j = i + 1U; j < higher.size(); ++j) {
        add_edge(higher[i], higher[j]);
      }
    }
  }

  utl::verify(seen.size() < kNoNode, "CCH edge count exceeds 32 bits");
  auto edges = std::vector<edge_ref>{};
  edges.reserve(seen.size());
  for (auto const key : seen) {
    auto const a = static_cast<std::uint32_t>(key >> 32U);
    auto const b = static_cast<std::uint32_t>(key);
    edges.push_back(rank[a] < rank[b] ? edge_ref{a, b} : edge_ref{b, a});
  }
  std::sort(begin(edges), end(edges), [](auto const& a, auto const& b) {
    return a.low_ == b.low_ ? a.high_ < b.high_ : a.low_ < b.low_;
  });

  auto h = header{};
  h.state_count_ = n_states;
  h.edge_count_ = edges.size();

  fmt::println("CCH: {} states, {} edges", n_states, edges.size());
  auto file =
      std::ofstream{artifact_path(out), std::ios::binary | std::ios::trunc};
  file.write(reinterpret_cast<char const*>(&h), sizeof(h));
  file.write(reinterpret_cast<char const*>(rank.data()),
             static_cast<std::streamsize>(rank.size() * sizeof(rank.front())));
  file.write(
      reinterpret_cast<char const*>(edges.data()),
      static_cast<std::streamsize>(edges.size() * sizeof(edges.front())));
  file.flush();
  utl::verify(file.good(), "could not write CCH topology");
}

namespace {
struct weighted_edge {
  cost_t forward_cost_{kInfeasible};
  cost_t backward_cost_{kInfeasible};
  std::uint32_t via_forward_{kNoNode};
  std::uint32_t via_backward_{kNoNode};
};

struct overlay {
  std::vector<weighted_edge> edges_;
};

using parameter_key = std::array<std::uint32_t, 6>;
using overlay_handle =
    std::pair<std::shared_lock<std::shared_mutex>, overlay const*>;

struct metric_entry {
  std::shared_mutex mutex_;
  parameter_key parameters_{};
  std::unique_ptr<overlay> overlay_;
};

struct metric_cache {
  std::mutex customization_mutex_;
  std::array<metric_entry, 2> entries_;
  std::size_t next_{};
};

struct topology {
  std::vector<std::uint32_t> offsets_;
  std::vector<std::uint32_t> rank_;
  std::vector<std::uint32_t> order_;
  std::vector<edge_ref> edges_;
  std::vector<std::vector<std::uint32_t>> up_;
  hash_map<std::uint64_t, std::uint32_t> edge_pos_;
};

cost_t add_cost(cost_t const a, cost_t const b) {
  if (a == kInfeasible || b == kInfeasible) {
    return kInfeasible;
  }
  auto const total = static_cast<std::uint64_t>(a) + b;
  return total < kInfeasible ? static_cast<cost_t>(total) : kInfeasible;
}

void relax(cost_t& current,
           std::uint32_t& via,
           cost_t const candidate,
           std::uint32_t const middle) {
  if (candidate < current) {
    current = candidate;
    via = middle;
  }
}

template <typename P, direction Dir>
std::unique_ptr<overlay> customize(typename P::parameters const& params,
                                   ways const& w,
                                   topology const& t) {
  // OSR's backward search has different penalty placement; customize it
  // separately instead of transposing the forward metric.
  auto const timer = utl::scoped_timer{"CCH customization"};
  auto o = std::make_unique<overlay>();
  o->edges_.resize(t.edges_.size());

  for_each_arc<P, Dir>(
      params, *w.r_, t.offsets_,
      [&](auto const from, auto const to, cost_t const cost) {
        auto const lo = t.rank_[from] < t.rank_[to] ? from : to;
        auto const hi = t.rank_[from] < t.rank_[to] ? to : from;
        auto const it = t.edge_pos_.find(edge_key(lo, hi));
        if (from == to) {
          return;
        }
        utl::verify(it != end(t.edge_pos_),
                    "CCH topology is missing an original arc");
        auto& e = o->edges_[it->second];
        auto& slot = from == lo ? e.forward_cost_ : e.backward_cost_;
        slot = std::min(slot, cost);
      });

  // Child edges are final when their lower-ranked endpoint is processed,
  // so stored witnesses remain valid and descend in rank when unpacked.
  for (auto const middle : t.order_) {
    auto const& incident = t.up_[middle];
    for (auto i = 0U; i < incident.size(); ++i) {
      auto const& e1 = o->edges_[incident[i]];
      auto const y = t.edges_[incident[i]].high_;
      for (auto j = i + 1U; j < incident.size(); ++j) {
        auto const& e2 = o->edges_[incident[j]];
        auto const z = t.edges_[incident[j]].high_;
        auto const lo = t.rank_[y] < t.rank_[z] ? y : z;
        auto const hi = t.rank_[y] < t.rank_[z] ? z : y;
        auto const it = t.edge_pos_.find(edge_key(lo, hi));
        utl::verify(it != end(t.edge_pos_),
                    "CCH topology is missing a fill edge");
        auto const& lo_edge = lo == y ? e1 : e2;
        auto const& hi_edge = hi == y ? e1 : e2;
        auto& target = o->edges_[it->second];
        relax(target.forward_cost_, target.via_forward_,
              add_cost(lo_edge.backward_cost_, hi_edge.forward_cost_), middle);
        relax(target.backward_cost_, target.via_backward_,
              add_cost(hi_edge.backward_cost_, lo_edge.forward_cost_), middle);
      }
    }
  }

  return o;
}

}  // namespace

struct cache::impl {
  std::once_flag loaded_;
  std::unique_ptr<topology> topology_;
  std::array<metric_cache, 4> metrics_;
};

cache::cache() : impl_{std::make_unique<impl>()} {}
cache::~cache() = default;

namespace {
bool load(ways const& w) {
  auto& c = *w.cch_->impl_;
  std::call_once(c.loaded_, [&] {
    auto h = header{};
    auto t = std::make_unique<topology>();
    t->offsets_ = make_offsets(*w.r_);
    if (!read_header(w.p_, h) || h.state_count_ != t->offsets_.back() ||
        !read_artifact(w.p_, h, t->rank_, t->edges_)) {
      return;
    }
    t->order_.resize(h.state_count_);
    t->up_.resize(h.state_count_);
    for (auto n = 0U; n != t->rank_.size(); ++n) {
      t->order_[t->rank_[n]] = n;
    }
    t->edge_pos_.reserve(t->edges_.size());
    for (auto id = 0U; id != t->edges_.size(); ++id) {
      auto const& e = t->edges_[id];
      t->up_[e.low_].push_back(id);
      t->edge_pos_.emplace(edge_key(e.low_, e.high_), id);
    }
    c.topology_ = std::move(t);
  });
  return c.topology_ != nullptr;
}

template <typename P, direction Dir>
auto get_overlay(typename P::parameters const& params, ways const& w) {
  auto& c = *w.cch_->impl_;
  constexpr auto slot = (std::is_same_v<P, bus> ? 2U : 0U) +
                        (Dir == direction::kBackward ? 1U : 0U);
  auto const key =
      parameter_key{params.uturn_penalty_,     params.private_gate_penalty_,
                    params.slow_turn_angle_,   params.sharp_turn_angle_,
                    params.slow_turn_penalty_, params.sharp_turn_penalty_};
  auto& metrics = c.metrics_[slot];
  auto const find = [&]() -> overlay_handle {
    for (auto& entry : metrics.entries_) {
      auto lock = std::shared_lock{entry.mutex_};
      if (entry.overlay_ != nullptr && entry.parameters_ == key) {
        return {std::move(lock), entry.overlay_.get()};
      }
    }
    return {};
  };
  if (auto hit = find(); hit.second != nullptr) {
    return hit;
  }
  auto const build_lock = std::unique_lock{metrics.customization_mutex_};
  if (auto hit = find(); hit.second != nullptr) {
    return hit;
  }
  // Build without locking cached overlays; active queries pin their entry
  // until reconstruction finishes. Two entries retain alternating metrics.
  auto o = customize<P, Dir>(params, w, *c.topology_);
  auto& entry = metrics.entries_[metrics.next_];
  {
    auto const write = std::unique_lock{entry.mutex_};
    entry.overlay_ = std::move(o);
    entry.parameters_ = key;
  }
  metrics.next_ = (metrics.next_ + 1U) % metrics.entries_.size();
  return find();
}

struct predecessor {
  std::uint32_t state_{kNoNode};
  way_candidate const* way_{};
  node_candidate const* node_{};
};

struct query_scratch {
  std::vector<cost_t> forward_;
  std::vector<cost_t> backward_;
  std::vector<std::uint32_t> touched_;
  std::vector<predecessor> f_pred_;
  std::vector<predecessor> b_pred_;
};

query_scratch& get_query_scratch() {
  static thread_local auto scratch = query_scratch{};
  return scratch;
}

using pq_t = std::priority_queue<std::pair<cost_t, std::uint32_t>,
                                 std::vector<std::pair<cost_t, std::uint32_t>>,
                                 std::greater<>>;

void add_start(std::vector<cost_t>& dist,
               std::vector<std::uint32_t>& touched,
               pq_t& pq,
               std::uint32_t const state,
               cost_t const cost) {
  if (cost < dist[state]) {
    dist[state] = cost;
    touched.push_back(state);
    pq.emplace(cost, state);
  }
}

template <typename P, direction Dir>
std::optional<path> query(typename P::parameters const& params,
                          ways const& w,
                          lookup const& l,
                          location const& from,
                          location const& to,
                          match_view_t const from_match,
                          match_view_t const to_match,
                          cost_t const max) {
  auto const [lock, overlay_ptr] = get_overlay<P, Dir>(params, w);
  auto const& o = *overlay_ptr;
  auto const& t = *w.cch_->impl_->topology_;
  auto const n_states = t.rank_.size();
  auto const& r = *w.r_;

  auto& [forward, backward, touched, f_pred, b_pred] = get_query_scratch();
  if (forward.size() != n_states) {
    touched.clear();
    forward.assign(n_states, kInfeasible);
    backward.assign(n_states, kInfeasible);
  }
  for (auto const s : touched) {
    forward[s] = kInfeasible;
    backward[s] = kInfeasible;
  }
  touched.clear();
  f_pred.resize(n_states);
  b_pred.resize(n_states);

  auto f_pq = pq_t{};
  auto b_pq = pq_t{};

  for (auto const& wc : from_match) {
    for (auto const* nc : {&wc.left_, &wc.right_}) {
      if (!nc->valid() || nc->node_ == node_idx_t::invalid() ||
          nc->cost_ >= max) {
        continue;
      }
      P::resolve_start_node(
          r, wc.way_, nc->node_, level_t{0.F}, Dir, [&](auto const n) {
            auto const id = state_id<P>(t.offsets_, n);
            if (nc->cost_ < forward[id]) {
              f_pred[id] = {kNoNode, &wc, nc};
              add_start(forward, touched, f_pq, id, nc->cost_);
            }
          });
    }
  }

  for (auto const& wc : to_match) {
    for (auto const* nc : {&wc.left_, &wc.right_}) {
      if (!nc->valid() || nc->node_ == node_idx_t::invalid() ||
          nc->cost_ >= max) {
        continue;
      }
      P::resolve_all(r, nc->node_, level_t{0.F}, [&](auto const n) {
        if (!P::is_dest_reachable(params, r, n, wc.way_,
                                  flip(opposite(Dir), nc->way_dir_), Dir)) {
          return;
        }
        auto const id = state_id<P>(t.offsets_, n);
        if (nc->cost_ < backward[id]) {
          b_pred[id] = {kNoNode, &wc, nc};
          add_start(backward, touched, b_pq, id, nc->cost_);
        }
      });
    }
  }

  auto best = max;
  auto meeting = kNoNode;

  auto const settle = [&](pq_t& pq, std::vector<cost_t>& own,
                          std::vector<cost_t> const& other, bool const fwd) {
    auto const [cost, n] = pq.top();
    pq.pop();
    if (cost != own[n] || cost >= best) {
      return;
    }
    if (other[n] != kInfeasible && add_cost(cost, other[n]) < best) {
      best = add_cost(cost, other[n]);
      meeting = n;
    }
    for (auto const edge_id : t.up_[n]) {
      auto const& e = o.edges_[edge_id];
      auto const high = t.edges_[edge_id].high_;
      auto const edge_cost = fwd ? e.forward_cost_ : e.backward_cost_;
      auto const next = add_cost(cost, edge_cost);
      if (next < best && next < own[high]) {
        own[high] = next;
        (fwd ? f_pred : b_pred)[high] = {n, nullptr, nullptr};
        touched.push_back(high);
        pq.emplace(next, high);
      }
    }
  };

  while (!f_pq.empty() || !b_pq.empty()) {
    auto const f_top = f_pq.empty() ? kInfeasible : f_pq.top().first;
    auto const b_top = b_pq.empty() ? kInfeasible : b_pq.top().first;
    // Both queue minima must reach the incumbent; their sum can stop too early.
    if (std::min(f_top, b_top) >= best) {
      break;
    }
    if (f_top <= b_top) {
      settle(f_pq, forward, backward, true);
    } else {
      settle(b_pq, backward, forward, false);
    }
  }

  if (meeting == kNoNode) {
    return std::nullopt;
  }
  auto states = std::vector<std::uint32_t>{meeting};
  while (f_pred[states.back()].state_ != kNoNode) {
    states.push_back(f_pred[states.back()].state_);
  }
  auto const start = f_pred[states.back()];
  std::reverse(begin(states), end(states));
  auto n = meeting;
  while (b_pred[n].state_ != kNoNode) {
    n = b_pred[n].state_;
    states.push_back(n);
  }
  auto const dest = b_pred[n];
  auto p = path{.cost_ = best};
  auto const connector = [&](predecessor const& pred, bool const reverse,
                             location const& loc) {
    auto const& nc = *pred.node_;
    p.dist_ += nc.dist_to_node_;
    p.segments_.push_back(
        {.polyline_ = l.get_node_candidate_path(*pred.way_, nc, reverse, loc),
         .from_level_ = nc.lvl_,
         .to_level_ = nc.lvl_,
         .from_ = reverse ? nc.node_ : node_idx_t::invalid(),
         .to_ = reverse ? node_idx_t::invalid() : nc.node_,
         .way_ = way_idx_t::invalid(),
         .cost_ = nc.cost_,
         .dist_ = static_cast<distance_t>(nc.dist_to_node_),
         .mode_ = mode::kCar});
  };
  connector(start, Dir == direction::kBackward, from);
  auto const decode = [&](std::uint32_t const state) {
    auto const it = std::upper_bound(begin(t.offsets_), end(t.offsets_), state);
    auto const node = static_cast<std::uint32_t>(it - begin(t.offsets_) - 1);
    auto const local = state - t.offsets_[node];
    return typename P::node{
        node_idx_t{node}, static_cast<way_pos_t>(local / 2U),
        local % 2U == 0U ? direction::kForward : direction::kBackward};
  };
  auto pending = std::vector<std::pair<std::uint32_t, std::uint32_t>>{};
  for (auto i = 1U; i < states.size(); ++i) {
    pending.emplace_back(states[i - 1U], states[i]);
    while (!pending.empty()) {
      auto const [a, b] = pending.back();
      pending.pop_back();
      auto const up = t.rank_[a] < t.rank_[b];
      auto const& e =
          o.edges_[t.edge_pos_.at(up ? edge_key(a, b) : edge_key(b, a))];
      auto const via = up ? e.via_forward_ : e.via_backward_;
      if (via != kNoNode) {
        pending.emplace_back(via, b);
        pending.emplace_back(a, via);
      } else {
        p.dist_ += add_path<P>(
            params, w, r, nullptr, nullptr, nullptr, decode(a), decode(b),
            up ? e.forward_cost_ : e.backward_cost_, p.segments_, Dir);
      }
    }
  }
  connector(dest, Dir == direction::kForward, to);
  if constexpr (Dir == direction::kBackward) {
    std::reverse(begin(p.segments_), end(p.segments_));
  }
  return p;
}

bool supports(search_profile const profile) {
  return profile == search_profile::kCar || profile == search_profile::kBus;
}
}  // namespace

bool usable(ways const& w,
            search_profile const profile,
            bitvec<node_idx_t> const* blocked,
            sharing_data const* sharing,
            elevation_storage const* elevations) {
  if (!supports(profile) || blocked != nullptr || sharing != nullptr ||
      elevations != nullptr) {
    return false;
  }
  return load(w);
}

std::optional<path> route(profile_parameters const& params,
                          ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          match_view_t const from_match,
                          match_view_t const to_match,
                          cost_t const max,
                          direction const dir) {
  if (!supports(profile) || !load(w) || from_match.empty() ||
      to_match.empty()) {
    return std::nullopt;
  }
  auto const run = [&]<typename P, direction Dir>() {
    return query<P, Dir>(std::get<typename P::parameters>(params), w, l, from,
                         to, from_match, to_match, max);
  };
  if (profile == search_profile::kCar) {
    return dir == direction::kForward
               ? run.template operator()<car, direction::kForward>()
               : run.template operator()<car, direction::kBackward>();
  }
  return dir == direction::kForward
             ? run.template operator()<bus, direction::kForward>()
             : run.template operator()<bus, direction::kBackward>();
}
}  // namespace osr::cch
