#pragma once

#include "utl/enumerate.h"
#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/routing/profile.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace cch {

// the general struct is inspired from the available
// dijkstra and a-star implementation in the repository
// to fit into the general program

template <osr::Profile P, bool EarlyTermination = false>
struct bidir_dijkstra {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;

  static constexpr auto const kDebug = false;

  struct get_bucket {
    osr::cost_t operator()(label const& l) { return l.cost(); }
  };

  void reset(osr::cost_t const max,
             osr::location const& start_loc,
             osr::location const& end_loc) {
    pq_f_.clear();
    pq_b_.clear();
    pq_f_.n_buckets(max + 1U);
    pq_b_.n_buckets(max + 1U);
    max_reached_f_ = false;
    max_reached_b_ = false;
    cost_f_.clear();
    cost_b_.clear();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    mu_ = osr::kInfeasible;
    curr_fw_cost_ = osr::cost_t{0U};
    curr_bw_cost_ = osr::cost_t{0U};
    meet_point_f_ = meet_point_f_.invalid();
    meet_point_b_ = meet_point_b_.invalid();
  }

  void add_start(osr::ways const& w, label const l) {
    if (cost_f_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                               node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "START ";
        l.get_node().print(std::cout, w);
        std::cout << " Importance: " << w.r_->node_importance_[l.get_node().n_];
        std::cout << "\n";
      }
      utl::verify(l.cost() < pq_f_.n_buckets(),
                  "bidir_dijkstra::add_start: label cost exceed max: {} >= {}",
                  l.cost(), pq_f_.n_buckets());
      pq_f_.push(l);
    }
  }

  void add_end(osr::ways const& w, label const l) {
    if (cost_b_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                               node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "END ";
        l.get_node().print(std::cout, w);
        std::cout << " Importance: " << w.r_->node_importance_[l.get_node().n_];
        std::cout << "\n";
      }
      utl::verify(l.cost() < pq_b_.n_buckets(),
                  "bidir_dijkstra::add_end: label cost exceed max: {} >= {}",
                  l.cost(), pq_b_.n_buckets());
      pq_b_.push(l);
    }
  }

  // Inpspired by the get_cost function of bidir
  template <osr::direction PathDir>
  osr::cost_t get_cost(node const n) const {
    if (PathDir == osr::direction::kForward) {
      auto const it = cost_f_.find(n.get_key());
      return it != end(cost_f_) ? it->second.cost(n) : osr::kInfeasible;
    } else {
      auto const it = cost_b_.find(n.get_key());
      return it != end(cost_b_) ? it->second.cost(n) : osr::kInfeasible;
    }
  }

  template <osr::direction PathDir>
  bool check_restrictions(osr::ways::routing const& r,
                          node const& curr,
                          osr::way_pos_t const& next_way_pos) {
    return r.is_restricted<PathDir, false>(curr.n_, curr.way_, next_way_pos);
  }

  template <osr::direction PathDir>
  void find_opposite(P::parameters const& params,
                     node const n,
                     osr::cost_t const curr_cost,
                     osr::ways::routing const& r) {
    for (auto const [way, idx] :
         utl::zip(r.node_ways_[n.n_], r.node_in_way_idx_[n.n_])) {
      auto const way_pos = r.get_way_pos(n.n_, way, idx);

      auto const check_op = [&](node const curr, node const contr) {
        auto contr_cost = get_cost<osr::opposite(PathDir)>(contr);
        if (contr_cost != osr::kInfeasible && curr_cost != osr::kInfeasible) {
          auto total_cost = static_cast<std::uint64_t>(curr_cost) +
                            static_cast<std::uint64_t>(contr_cost);

          if (curr.way_ == contr.way_ &&
              curr.dir_ == osr::opposite(contr.dir_)) {
            total_cost += static_cast<std::uint64_t>(params.uturn_penalty_);
          }

          if (static_cast<osr::cost_t>(total_cost) < mu_) {
            if (PathDir == osr::direction::kForward) {
              meet_point_f_ = curr;
              meet_point_b_ = contr;
            } else {
              meet_point_f_ = contr;
              meet_point_b_ = curr;
            }

            mu_ = static_cast<osr::cost_t>(total_cost);
            if constexpr (kDebug) {
              std::cout << "=> MEETING POINT: " << n.n_
                        << " TOTAL COST: " << mu_ << "\n";
              std::cout << " CURR_COST: " << curr_cost
                        << " CONTR_COST: " << contr_cost << "\n";
            }
          }
        }

        return;
      };

      if (check_restrictions<PathDir>(r, n, way_pos)) {
        continue;
      }

      auto const op_node_fw = node{n.n_, way_pos, osr::direction::kForward};
      check_op(n, op_node_fw);

      auto const op_node_bw = node{n.n_, way_pos, osr::direction::kBackward};
      check_op(n, op_node_bw);
    }
  }

  template <osr::direction SearchDir, bool WithBlocked, osr::direction PathDir>
  bool run_single(P::parameters const& params,
                  osr::ways const& w,
                  osr::ways::routing const& r,
                  osr::cost_t const max,
                  osr::dial<label, get_bucket>& pq,
                  cost_map& costs) {
    auto const is_fwd = PathDir == osr::direction::kForward;
    auto const l = pq.pop();

    auto const curr = l.get_node();
    auto const curr_cost = get_cost<PathDir>(curr);

    if (is_fwd) {
      curr_fw_cost_ = curr_cost;
    } else {
      curr_bw_cost_ = curr_cost;
    }

    if (get_cost<PathDir>(l.get_node()) < l.cost()) {
      if constexpr (kDebug) {
        is_fwd ? std::cout << "RETURN (fw) " : std::cout << "RETURN (bw) ";
        curr.print(std::cout, w);
        std::cout << " Expected " << get_cost<PathDir>(l.get_node())
                  << " but got " << l.cost() << "\n";
      }
      return PathDir == osr::direction::kForward ? !max_reached_f_
                                                 : !max_reached_b_;
    }

    if constexpr (kDebug) {
      is_fwd ? std::cout << "EXTRACT (fw) " : std::cout << "EXTRACT (bw) ";
      l.get_node().print(std::cout, w);
      std::cout << " Importance: " << r.node_importance_[l.get_node().n_];
      std::cout << " COST: " << get_cost<PathDir>(l.get_node());
      std::cout << " PQ SIZE: " << pq.size();
      std::cout << "\n";
    }

    auto const& curr_importance = r.node_importance_[curr.n_];
    auto targets = r.sc_targets_[curr_importance];
    osr::vec<osr::node_idx_t> self_targets = {};
    self_targets.resize(r.cch_cost_self_[curr_importance].size(), curr.n_);
    auto sc_costs = is_fwd ? r.cch_cost_up_[curr_importance]
                           : r.cch_cost_down_[curr_importance];
    auto const sc_costs_self = r.cch_cost_self_[curr_importance];
    auto sc_properties = is_fwd ? r.cch_sc_up_[curr_importance]
                                : r.cch_sc_down_[curr_importance];
    auto const sc_properties_self = r.cch_sc_self_[curr_importance];

    sc_costs.insert(sc_costs.end(), sc_costs_self.begin(), sc_costs_self.end());
    sc_properties.insert(sc_properties.end(), sc_properties_self.begin(),
                         sc_properties_self.end());
    targets.insert(targets.end(), self_targets.begin(), self_targets.end());
    utl::verify(
        sc_costs.size() == sc_properties.size() &&
            sc_costs.size() == targets.size(),
        "[BIDIR] Unequal size of costs ({}), targets ({}) and shortcuts ({})",
        sc_costs.size(), targets.size(), sc_properties.size());

    // add all shortcuts to the queue:
    for (auto [target, cost, property] :
         utl::zip(targets, sc_costs, sc_properties)) {
      if (cost == osr::kInfeasible) {
        if constexpr (kDebug) {
          std::cout << "  REJECTED: " << target << " with infeasible cost\n";
        }
        continue;
      }

      auto const& entry_node =
          is_fwd ? property.entry_node_ : property.exit_node_;
      auto const& exit_node =
          is_fwd ? property.exit_node_ : property.entry_node_;

      if (check_restrictions<PathDir>(r, curr, entry_node.way_)) {
        if constexpr (kDebug) {
          std::cout << "  REJECTED: " << target << " with restriction\n";
        }
        // add possible u turn here to enter the shortcut correctly
        continue;
      }
      utl::verify(target == exit_node.n_, "Got target: {} but exptected: {}",
                  exit_node.n_, target);
      utl::verify(curr.n_ == entry_node.n_, "Got entry: {} but expected: {}",
                  entry_node.n_, curr.n_);

      auto neighbor_cost =
          osr::clamp_cost(static_cast<std::uint64_t>(cost) + curr_cost);
      if (curr.way_ == entry_node.way_ &&
          curr.dir_ == osr::opposite(entry_node.dir_)) {
        neighbor_cost += params.uturn_penalty_;
      }

      if (neighbor_cost >= max && is_fwd) {
        max_reached_f_ = true;
        break;
      }
      if (neighbor_cost >= max && !is_fwd) {
        max_reached_b_ = true;
        break;
      }

      // update new neighbor costs:
      auto const neighbor =
          typename P::node{target, exit_node.way_, exit_node.dir_};
      if (costs[neighbor.get_key()].update(
              l, neighbor, static_cast<osr::cost_t>(neighbor_cost), curr)) {
        auto next = label{neighbor, static_cast<osr::cost_t>(neighbor_cost)};
        next.track(l, r, r.node_ways_[target][exit_node.way_],
                   neighbor.get_node(), false);

        auto const hashmap_cost = costs.find(neighbor.get_key());
        utl::verify(hashmap_cost->second.cost(neighbor) == neighbor_cost,
                    "Expected costs {} but got {}", neighbor_cost,
                    hashmap_cost->second.cost(neighbor));
        utl::verify(get_cost<PathDir>(neighbor) == neighbor_cost,
                    "Expected costs {} but got {}", neighbor_cost,
                    get_cost<PathDir>(neighbor));

        pq.push(std::move(next));

        if constexpr (kDebug) {
          std::cout << "  ";
          neighbor.print(std::cout, w);
          is_fwd ? std::cout << " -> PUSH (fw)" : std::cout << " -> PUSH (bw)";
          std::cout << " PQ SIZE: " << pq.size() << " COST: " << neighbor_cost
                    << "\n";
        }
      } else {
        if constexpr (kDebug) {
          std::cout << "  ";
          neighbor.print(std::cout, w);
          is_fwd ? std::cout << " -> DOMINATED (fw)\n"
                 : std::cout << " -> DOMINATED (bw)\n";
        }
      }

      // check contrary cost and potential meetpoint:
      find_opposite<PathDir>(params, neighbor, neighbor_cost, r);
    }

    return SearchDir == osr::direction::kForward ? !max_reached_f_
                                                 : !max_reached_b_;
  }

  template <osr::direction SearchDir, bool WithBlocked>
  bool run(P::parameters const& params,
           osr::ways const& w,
           osr::ways::routing const& r,
           osr::cost_t const max,
           osr::bitvec<osr::node_idx_t> const* blocked,
           osr::sharing_data const* sharing,
           osr::elevation_storage const* elevations) {
    if (blocked != nullptr || sharing != nullptr || elevations != nullptr) {
      std::cout << "[WARNING] This implementation of CCH Bidir Dijkstra does "
                   "not support blocked, sharing and elevations\n";
    }

    // always run the side with the cheapest next neighbor if both queues are
    // not empty:
    while (!pq_f_.empty() || !pq_b_.empty()) {

      if (!pq_f_.empty() && curr_fw_cost_ <= curr_bw_cost_) {
        if (!run_single<SearchDir, WithBlocked, osr::direction::kForward>(
                params, w, r, max, pq_f_, cost_f_)) {
          break;
        }
      } else if (!pq_b_.empty() && curr_fw_cost_ > curr_bw_cost_) {
        if (!run_single<SearchDir, WithBlocked, osr::direction::kBackward>(
                params, w, r, max, pq_b_, cost_b_)) {
          break;
        }
      } else if (pq_f_.empty() && !pq_b_.empty()) {
        if (!run_single<SearchDir, WithBlocked, osr::direction::kBackward>(
                params, w, r, max, pq_b_, cost_b_)) {
          break;
        }
      } else if (!pq_f_.empty() && pq_b_.empty()) {
        if (!run_single<SearchDir, WithBlocked, osr::direction::kForward>(
                params, w, r, max, pq_f_, cost_f_)) {
          break;
        }
      }

      // While reformulating the while loop I run into a dead lock.
      // Used [AI] here to confirm my assumptions. It recommended to use
      // else if statements instead of single if statements. The AI's
      // recommended solution was logically incorrest but I used the idea of
      // else if statements for my fix instead of iterative if statements

      if (static_cast<std::uint64_t>(curr_fw_cost_) + curr_bw_cost_ >= mu_) {
        if constexpr (kDebug) {
          std::cout << "TERMINATED: cost(fn) + cost(bn) >= mu\n";
        }
        return false;
      }
    }
    if constexpr (kDebug) {
      std::cout << "TERMINATED: empty priority queues. ";
      std::cout << "Size Forward Queue: " << pq_f_.size() << " ";
      std::cout << "Size Backward Queue: " << pq_b_.size() << "\n";
    }

    return !max_reached_f_ || !max_reached_b_;
  }

  bool run(P::parameters const& params,
           osr::ways const& w,
           osr::ways::routing const& r,
           osr::cost_t const max,
           osr::bitvec<osr::node_idx_t> const* blocked,
           osr::sharing_data const* sharing,
           osr::elevation_storage const* elevations,
           osr::direction const dir) {
    if (blocked == nullptr) {
      return dir == osr::direction::kForward
                 ? run<osr::direction::kForward, false>(
                       params, w, r, max, blocked, sharing, elevations)
                 : run<osr::direction::kBackward, false>(
                       params, w, r, max, blocked, sharing, elevations);
    } else {
      return dir == osr::direction::kForward
                 ? run<osr::direction::kForward, true>(
                       params, w, r, max, blocked, sharing, elevations)
                 : run<osr::direction::kBackward, true>(
                       params, w, r, max, blocked, sharing, elevations);
    }
  }

  osr::location start_loc_;
  osr::location end_loc_;
  osr::dial<label, get_bucket> pq_f_{get_bucket{}};
  osr::dial<label, get_bucket> pq_b_{get_bucket{}};
  ankerl::unordered_dense::map<key, entry, hash> cost_f_;
  ankerl::unordered_dense::map<key, entry, hash> cost_b_;
  osr::cost_t mu_;
  osr::cost_t curr_fw_cost_;
  osr::cost_t curr_bw_cost_;
  node meet_point_f_;
  node meet_point_b_;
  bool max_reached_f_{};
  bool max_reached_b_{};
};
}  // namespace cch