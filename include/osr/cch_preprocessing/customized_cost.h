#pragma once

#include <algorithm>
#include <filesystem>
#include <iterator>
#include <list>
#include <optional>
#include <stack>
#include <utility>
#include <vector>

#include "fmt/core.h"
#include "fmt/std.h"

#include "cista/memory_holder.h"
#include "cista/reflection/comparable.h"

#include "utl/progress_tracker.h"

#include "osr/cch_preprocessing/elimination_tree.h"
#include "osr/cch_preprocessing/node_ordering.h"
#include "osr/cch_preprocessing/profile_name.h"
#include "osr/elevation_storage.h"
#include "osr/routing/profile.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr::cch_preprocessing {

template <Profile P>
struct customized_cost_stored {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;

  // TODO: make this a constexpr static member of the profile
  static constexpr auto kFilename = profile_name<P>::value;

  std::vector<ext_edge> trace_sg_edge(ext_edge_idx_t const& edge_idx) const {
    std::vector<ext_edge> res{0};
    std::stack<std::uint32_t> st{};
    st.push(edge_idx.v_);
    while (!st.empty()) {
      auto const uv = st.top();
      auto const edge = extended_edges_.at(uv);
      st.pop();

      if (edge.is_original()) {
        res.emplace_back(edge);
      } else {
        auto const first_half = edge.traceback_.first;
        auto const second_half = edge.traceback_.second;
        utl::verify(get_edge(first_half).cost_ + get_edge(second_half).cost_ ==
                        edge.cost_,
                    "traceback cost mismatch, from tuple {}, {} and {}", uv,
                    first_half, second_half);

        utl::verify(edge.from_ == get_edge(first_half).from_ &&
                        edge.to_ == get_edge(second_half).to_,
                    "traceback from/to mismatch, from tuple {}, {} and {}", uv,
                    first_half, second_half);

        utl::verify(get_edge(first_half).to_ == get_edge(second_half).from_,
                    "traceback transit mismatch, from tuple {}, {} and {}", uv,
                    first_half, second_half);

        st.push(second_half.v_);
        st.push(first_half.v_);
      }
    }
    return res;
  }

  node_idx_t const& get_parent(std::uint32_t const idx) const {
    utl::verify(idx < specialized_elimination_tree_.size(),
                "node index {} out of bounds, max {}", idx,
                specialized_elimination_tree_.size());
    return specialized_elimination_tree_.at(idx);
  }

  ext_edge get_edge(ext_edge_idx_t const& idx) const {
    utl::verify(idx.v_ < extended_edges_.size(),
                "edge index {} out of bounds, max {}", idx,
                extended_edges_.size());
    return extended_edges_.at(idx.v_);
  }

  ext_node get_virtual_node_idx(node const& n,
                                node_ordering const& ordering) const {
    if (n.get_node() == node_idx_t::invalid() ||
        ordering.get_ordering(n.get_node()) >=
            static_cast<node_idx_t>(virtual_nodes_.size())) {
      return ext_node::invalid();
    }

    auto const& vnodes = virtual_nodes_[ordering.get_ordering(n.get_node()).v_];
    auto it = std::lower_bound(vnodes.begin(), vnodes.end(), n);

    if (it == vnodes.end() || !(*it == n)) {
      return ext_node::invalid();
    }

    return ext_node{.primary_idx_ = ordering.get_ordering(n.get_node()),
                    .sub_idx_ = static_cast<std::uint16_t>(
                        std::distance(vnodes.begin(), it))};
  }

  node get_virtual_node(ext_node const& idx) const {
    utl::verify(idx.first() < virtual_nodes_.size(),
                "node index {} out of bounds, max {}", idx.first(),
                virtual_nodes_.size());
    utl::verify(idx.second() < virtual_nodes_[idx.first().v_].size(),
                "sub-node index {} out of bounds, max {}", idx.second(),
                virtual_nodes_[idx.first().v_].size());
    return virtual_nodes_[idx.first().v_][idx.second()];
  }

  inline std::uint32_t get_flatten_node_idx(ext_node const idx) const {
    return static_cast<std::uint32_t>(idx_ranges_.at(idx.first().v_) +
                                      idx.second());
  }

  template <bool Upward, typename Fn>
  void for_each_edge(std::uint32_t const idx, Fn&& f) const {
    if (Upward) {
      utl::verify(idx < upward_edges_.size(),
                  "upward edges index {} out of bounds, max {}", idx,
                  upward_edges_.size());
      for (std::uint32_t edge_idx = upward_edges_[idx].first;
           edge_idx < upward_edges_[idx].second; ++edge_idx) {
        f(edge_idx, extended_edges_.at(edge_idx));
      }
    } else {
      utl::verify(idx < downward_edges_.size(),
                  "downward edges index {} out of bounds, max {}", idx,
                  downward_edges_.size());
      for (std::uint32_t edge_idx = downward_edges_[idx].first;
           edge_idx < downward_edges_[idx].second; ++edge_idx) {
        f(edge_idx, extended_edges_.at(edge_idx));
      }
    }
  }

  template <std::size_t NMaxTypes>
  friend constexpr auto static_type_hash(
      customized_cost_stored<P> const*,
      cista::hash_data<NMaxTypes> h) noexcept {
    using cista::static_type_hash;
    h = h.combine(cista::hash("customized_cost_stored v1.0"));
    h = static_type_hash(cista::null<P>(), h);
    return h;
  }

  static cista::wrapped<customized_cost_stored<P>> read(
      std::filesystem::path const& path) {
    return cista::read<customized_cost_stored<P>>(
        path / std::filesystem::path{kFilename});
  }

  void write(std::filesystem::path const& path) const {
    return cista::write(path / std::filesystem::path{kFilename}, *this);
  }

  vecvec<std::uint32_t, node> virtual_nodes_;
  vec<std::uint32_t> idx_ranges_;
  vec<node_idx_t> specialized_elimination_tree_;
  vec<ext_edge> extended_edges_;
  vec<pair<std::uint32_t, std::uint32_t>> upward_edges_, downward_edges_;
};

template <Profile P>
struct customized_cost_builder {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;

  using list_it = std::list<ext_edge_idx_t>::iterator;

  static void initialize(std::size_t const num_nodes) {
    extended_edges_.clear();
    upward_edges_.resize(num_nodes);
    downward_edges_.resize(num_nodes);
  }

  static std::list<ext_edge_idx_t>& get_upward_edges(ext_node const& idx) {
    utl::verify(idx.first().v_ < upward_edges_.size(),
                "node index {} out of bounds, max {}", idx.first().v_,
                upward_edges_.size());
    utl::verify(idx.second() < upward_edges_[idx.first().v_].size(),
                "sub-node index {} out of bounds, max {}", idx.second(),
                upward_edges_[idx.first().v_].size());
    return upward_edges_[idx.first().v_][idx.second()];
  }

  static std::list<ext_edge_idx_t>& get_downward_edges(ext_node const& idx) {
    utl::verify(idx.first().v_ < downward_edges_.size(),
                "node index {} out of bounds, max {}", idx.first().v_,
                downward_edges_.size());
    utl::verify(idx.second() < downward_edges_[idx.first().v_].size(),
                "sub-node index {} out of bounds, max {}", idx.second(),
                downward_edges_[idx.first().v_].size());
    return downward_edges_[idx.first().v_][idx.second()];
  }

  static ext_edge& get_edge(ext_edge_idx_t const idx) {
    utl::verify(idx.v_ < extended_edges_.size(),
                "edge index {} out of bounds, max {}", idx.v_,
                extended_edges_.size());
    return extended_edges_[idx.v_];
  }

  static ext_edge_idx_t const get_rev_edge(ext_edge_idx_t const& uv) {
    return static_cast<ext_edge_idx_t>(uv.v_ ^ 1);
  }

  static void add_edge(ext_node const& from_idx,
                       ext_node const& to_idx,
                       std::optional<list_it> from_ins_pos = std::nullopt) {
    if (!from_ins_pos)
      from_ins_pos = std::make_optional(get_upward_edges(from_idx).end());

    auto curr_size = static_cast<ext_edge_idx_t>(extended_edges_.size());
    get_upward_edges(from_idx).emplace(*from_ins_pos, curr_size);
    extended_edges_.emplace_back(ext_edge{.from_ = from_idx, .to_ = to_idx});
    curr_size++;
    get_downward_edges(to_idx).emplace_back(curr_size);
    extended_edges_.emplace_back(ext_edge{.from_ = to_idx, .to_ = from_idx});
  }

  static void build_node_mapping(ways::routing const& r,
                                 node_ordering const& ordering,
                                 customized_cost_stored<P>& ccs) {
    ccs.virtual_nodes_.resize(ordering.size());
    ccs.idx_ranges_.resize(ordering.size());

    // map old ordering to new ordering
    ordering.for_each_node<true>(
        [&](node_idx_t const& from_ord, node_idx_t const& from) {
          P::resolve_all(r, from, kNoLevel, [&](node const n) {
            ccs.virtual_nodes_[from_ord.v_].push_back(n);
          });
          utl::sort(ccs.virtual_nodes_[from_ord.v_]);
        });

    std::uint32_t range_start = 0;
    ordering.for_each_node<true>([&](node_idx_t const& from_ord,
                                     node_idx_t const&) {
      upward_edges_[from_ord.v_].resize(ccs.virtual_nodes_[from_ord.v_].size());
      downward_edges_[from_ord.v_].resize(
          ccs.virtual_nodes_[from_ord.v_].size());

      std::fill(upward_edges_[from_ord.v_].begin(),
                upward_edges_[from_ord.v_].end(), std::list<ext_edge_idx_t>{});
      std::fill(downward_edges_[from_ord.v_].begin(),
                downward_edges_[from_ord.v_].end(),
                std::list<ext_edge_idx_t>{});

      ccs.idx_ranges_[from_ord.v_] = range_start;
      range_start +=
          static_cast<std::uint32_t>(ccs.virtual_nodes_[from_ord.v_].size());
    });
  }

  static void transfer_edges_cost(ways const& w,
                                  node_ordering const& ordering,
                                  customized_cost_stored<P>& ccs,
                                  typename P::parameters const& params) {

    for (std::uint32_t i = 0U; i < ccs.virtual_nodes_.size(); ++i) {
      for (std::uint16_t j = 0; j < ccs.virtual_nodes_[i].size(); ++j) {
        auto const from_idx = ext_node::to_ext_node(i, j);
        auto const from = ccs.virtual_nodes_[i][j];

        P::template adjacent<direction::kForward, false>(
            params, *w.r_, from, nullptr, nullptr, nullptr,
            [&](node const to, std::uint32_t const cost, distance_t,
                way_idx_t const, std::uint16_t, std::uint16_t,
                elevation_storage::elevation, bool const) {
              utl::verify(
                  ccs.get_virtual_node_idx(to, ordering) != ext_node::invalid(),
                  "Virtual node index for node {} not found", to.get_node());
              auto const to_idx = ccs.get_virtual_node_idx(to, ordering);
              if (from_idx < to_idx) {
                add_edge(from_idx, to_idx);
                get_edge(get_upward_edges(from_idx).back()).update(cost);
              } else if (to_idx < from_idx) {
                bool found = false;
                for (auto const& adj_edge : get_downward_edges(from_idx)) {
                  if (get_edge(adj_edge).to_ == to_idx) {
                    get_edge(adj_edge).update(cost);
                    found = true;
                    break;
                  }
                }
                if (!found) {
                  add_edge(to_idx, from_idx);
                  get_edge(get_downward_edges(from_idx).back()).update(cost);
                }
              }
            });
      }
    }
  }

  static void customize(customized_cost_stored<P>& ccs) {
    auto const feasible_check = [&](ext_edge_idx_t const& uv,
                                    ext_edge_idx_t const& vw) -> bool {
      return get_edge(uv).cost_ != kInfeasible &&
             get_edge(vw).cost_ != kInfeasible;
    };

    auto const add_cost = [&](ext_edge_idx_t const& uv,
                              ext_edge_idx_t const& vw,
                              ext_edge_idx_t const& uw) {
      if (!feasible_check(uv, vw) ||
          kInfeasible - get_edge(uv).cost_ <= get_edge(vw).cost_) {
        return;
      }
      auto const new_cost = get_edge(uv).cost_ + get_edge(vw).cost_;
      get_edge(uw).update(new_cost, uv, vw);
    };

    for (std::uint32_t i = 0U; i < ccs.virtual_nodes_.size(); ++i) {
      for (std::uint16_t j = 0; j < ccs.virtual_nodes_[i].size(); ++j) {
        auto const u_idx = ext_node::to_ext_node(i, j);
        // auto const u = ccs.virtual_nodes_[i][j];

        get_upward_edges(u_idx).sort(
            [&](ext_edge_idx_t const& a, ext_edge_idx_t const& b) {
              auto const& ato = get_edge(a).to_;
              auto const& bto = get_edge(b).to_;
              return ato < bto;
            });

        for (auto const& uv : get_downward_edges(u_idx)) {
          if (!feasible_check(get_rev_edge(uv), get_rev_edge(uv)) &&
              !feasible_check(uv, uv)) {
            continue;
          }

          auto const v_idx = get_edge(uv).to_;
          auto const& v_edges = get_upward_edges(v_idx);
          auto u_edge_it = get_upward_edges(u_idx).end();

          if (v_edges.empty()) {
            continue;
          }
          for (auto v_edge_it = v_edges.end(); v_edge_it != v_edges.begin();
               v_edge_it = std::prev(v_edge_it)) {
            auto const vw = *std::prev(v_edge_it);
            auto const w_idx = get_edge(vw).to_;

            if (u_idx >= w_idx) {
              break;
            }

            if (!feasible_check(uv, vw) &&
                !feasible_check(get_rev_edge(vw), get_rev_edge(uv))) {
              continue;
            }

            while (!get_upward_edges(u_idx).empty() &&
                   u_edge_it != get_upward_edges(u_idx).begin() &&
                   w_idx < get_edge(*std::prev(u_edge_it)).to_) {
              u_edge_it = std::prev(u_edge_it);
            }
            if (get_upward_edges(u_idx).empty() ||
                u_edge_it == get_upward_edges(u_idx).begin() ||
                get_edge(*std::prev(u_edge_it)).to_ < w_idx) {
              add_edge(u_idx, w_idx, u_edge_it);
            }

            auto const uw = *std::prev(u_edge_it);

            add_cost(uv, vw, uw);
            add_cost(get_rev_edge(vw), get_rev_edge(uv), get_rev_edge(uw));
          }
        }
      }
    }
  }

  static customized_cost_stored<P> build(ways const& w,
                                         node_ordering const& ordering,
                                         elimination_tree const& et,
                                         typename P::parameters const& params) {
    auto pt =
        utl::get_active_progress_tracker_or_activate("osr-cch-preprocess");
    customized_cost_stored<P> ccs;
    initialize(ordering.size());

    pt->status("Build virtual node mapping").in_high(1).out_bounds(0, 10);
    build_node_mapping(*w.r_, ordering, ccs);
    pt->update(1);

    pt->status("Transfer edges cost").in_high(1).out_bounds(10, 30);
    transfer_edges_cost(w, ordering, ccs, params);
    pt->update(1);

    pt->status("Customizing cost").in_high(1).out_bounds(30, 65);
    customize(ccs);
    pt->update(1);

    pt->status("Build specialized elimination tree")
        .in_high(1)
        .out_bounds(65, 70);
    ccs.specialized_elimination_tree_.resize(
        ccs.idx_ranges_.back() +
        static_cast<std::uint32_t>(ccs.virtual_nodes_.back().size()));
    for (std::uint32_t i = 0U; i < ccs.virtual_nodes_.size(); ++i) {
      for (std::uint16_t j = 0; j < ccs.virtual_nodes_[i].size(); ++j) {
        auto const idx = ccs.idx_ranges_[i] + j;
        if (j != ccs.virtual_nodes_[i].size() - 1) {
          ccs.specialized_elimination_tree_[idx] =
              static_cast<node_idx_t>(ccs.idx_ranges_[i] + j + 1);
        } else {
          auto const par = et.tree_.at(static_cast<node_idx_t>(i));
          if (par != node_idx_t::invalid()) {
            auto const par_idx = ccs.idx_ranges_[par.v_];
            ccs.specialized_elimination_tree_[idx] =
                static_cast<node_idx_t>(par_idx);
          } else {
            ccs.specialized_elimination_tree_[idx] = node_idx_t::invalid();
          }
        }
      }
    }
    pt->update(1);

    pt->status("Store customized cost").in_high(1).out_bounds(70, 80);
    std::vector<std::uint32_t> new_edge_idxs = {};
    new_edge_idxs.resize(extended_edges_.size());
    for (std::uint32_t i = 0U; i < ccs.virtual_nodes_.size(); ++i) {
      for (std::uint16_t j = 0U; j < ccs.virtual_nodes_[i].size(); ++j) {
        std::uint32_t start = ccs.extended_edges_.size();
        std::uint32_t end =
            start + static_cast<std::uint32_t>(upward_edges_[i][j].size());
        ccs.upward_edges_.emplace_back(pair{start, end});
        for (auto const& edge_idx : upward_edges_[i][j]) {
          new_edge_idxs[edge_idx.v_] = ccs.extended_edges_.size();
          ccs.extended_edges_.emplace_back(get_edge(edge_idx));
        }

        start = ccs.extended_edges_.size();
        end = start + static_cast<std::uint32_t>(downward_edges_[i][j].size());
        ccs.downward_edges_.emplace_back(pair{start, end});
        for (auto const& edge_idx : downward_edges_[i][j]) {
          new_edge_idxs[edge_idx.v_] = ccs.extended_edges_.size();
          ccs.extended_edges_.emplace_back(get_edge(edge_idx));
        }
      }
    }
    for (auto& edge : ccs.extended_edges_) {
      if (!edge.is_original()) {
        edge.traceback_.first = static_cast<ext_edge_idx_t>(
            new_edge_idxs[edge.traceback_.first.v_]);
        edge.traceback_.second = static_cast<ext_edge_idx_t>(
            new_edge_idxs[edge.traceback_.second.v_]);
      }
    }
    pt->update(1);

    return ccs;
  }

  inline static std::vector<ext_edge> extended_edges_ = {};
  inline static std::vector<std::vector<std::list<ext_edge_idx_t>>>
      upward_edges_ = {},
      downward_edges_ = {};
};

}  // namespace osr::cch_preprocessing