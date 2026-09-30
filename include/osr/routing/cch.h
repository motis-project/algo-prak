#pragma once

#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

#include "osr/elevation_storage.h"
#include "osr/routing/profile.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

template <Profile P>
struct cch {

  struct shortcut {
    /* from*/
    typename P::node to_;
    typename P::node inbetween_up_;
    typename P::node inbetween_down_;
    cost_t weight_up_{kInfeasible};  // up and down for directions
    cost_t weight_down_{kInfeasible};
    bool is_shortcut_up_{false};
    bool is_shortcut_down_{false};
  };

  struct item {
    typename P::node n_;
    std::uint32_t importance_;
  };

  struct route {
    cost_t cost_{kInfeasible};
    typename P::node intersection_{};
    // pairs upgraph edges (to unpack)
    std::vector<typename P::node> packed_path_;
  };

  struct pq_entry {
    cost_t cost_;
    typename P::node node_;

    bool operator>(pq_entry const& other_entry) const {
      return cost_ > other_entry.cost_;
    }
  };

  osr::hash_map<typename P::node, std::uint32_t> rank_;

  osr::hash_map<typename P::node, std::vector<shortcut>> up_graph_;

  std::size_t n_nodes_{};  // topology dataset

  void build_pre_order(ways const& way) {
    std::vector<item> nodes;
    for (node_idx_t n = node_idx_t{0}; n < way.n_nodes(); ++n) {
      auto const importance = way.r_->node_importance_[n];
      P::resolve_all(*way.r_, n, level_t{}, [&](typename P::node const vn) {
        nodes.push_back(item{vn, importance});
      });
    }
    std::sort(nodes.begin(), nodes.end(), [&](item const& a, item const& b) {
      if (a.importance_ != b.importance_) {
        return a.importance_ < b.importance_;
      }
      return a.n_ < b.n_;  // guaranteed no tie
    });

    std::uint32_t rank = 0;
    for (auto const& x : nodes) {
      rank_[x.n_] = rank++;
    }
  }

  template <direction SearchDir, typename Fn>
  void for_each_edge(ways const& way,
                     typename P::parameters const& params,
                     Fn&& fn) const {
    for (node_idx_t n = node_idx_t{}; n < way.n_nodes(); ++n) {
      P::resolve_all(*way.r_, n, level_t{}, [&](typename P::node const u) {
        P::template adjacent<SearchDir, false>(
            params, *way.r_, u,
            nullptr,  // blocked
            nullptr,  // additional
            nullptr,  // elevation_storage
            [&](typename P::node const v, cost_t const cost,
                distance_t /*dist*/, way_idx_t /*way*/, int /*0*/, int /*0*/,
                elevation_storage::elevation /*elevation*/, bool /*false*/
            ) { fn(u, v, cost); });
      });
    }
  }

  std::vector<typename P::node> get_ordered_nodes() {
    std::vector<typename P::node> ordered_nodes(rank_.size());
    for (auto const& pair : rank_) {
      ordered_nodes.at(pair.second) = pair.first;
    }
    return ordered_nodes;
  }

  // tpopolgy
  template <direction SearchDir>
  void build_topology(ways const& way) {
    build_pre_order(way);
    n_nodes_ = way.n_nodes();

    osr::hash_map<typename P::node, std::vector<typename P::node>> adj_temp;

    for_each_edge<SearchDir>(
        way, typename P::parameters{},
        [&](typename P::node const u, typename P::node v, cost_t /*cost*/) {
          if (u == v) {
            return;
          }  // self edge
          adj_temp[u].push_back(v);
          adj_temp[v].push_back(u);
        });

    auto const ordered_nodes = get_ordered_nodes();

    for (typename P::node const v : ordered_nodes) {
      if (!up_graph_.contains(v)) {
        up_graph_[v] = std::vector<shortcut>{};
      }
    }

    for (typename P::node const v : ordered_nodes) {
      auto const rank_v = rank_.at(v);
      std::vector<typename P::node> upward_neighbour;
      for (auto neighbour : adj_temp[v]) {
        if (rank_.at(neighbour) > rank_v) {
          bool duplicate(false);
          for (typename P::node const duplicates : upward_neighbour) {
            if (duplicates == neighbour) {
              duplicate = true;
              break;
            }
          }
          if (!duplicate) {
            upward_neighbour.push_back(neighbour);
          }
        }
      }
      // non-shortcut roads
      auto& v_up_edges = up_graph_.at(v);
      for (auto const u : upward_neighbour) {
        v_up_edges.push_back({u, typename P::node{}, typename P::node{},
                              kInfeasible, kInfeasible, false, false});
      }

      // shortcuts
      for (std::size_t i = 0; i < upward_neighbour.size(); ++i) {
        auto u = upward_neighbour[i];

        for (auto j = i + 1; j < upward_neighbour.size(); ++j) {
          auto w = upward_neighbour[j];

          bool is_duplicate = false;
          for (auto const& existing_neighbour : adj_temp.at(u)) {
            if (existing_neighbour == w) {
              is_duplicate = true;
              break;
            }
          }
          if (!is_duplicate) {
            adj_temp[u].push_back(w);
            adj_temp[w].push_back(u);
          }
        }
      }
      adj_temp.erase(v);
    }
  }

  // phase2
  template <direction SearchDir>
  void customise(ways const& way, typename P::parameters const& params) {
    for (auto& [node, edges] : up_graph_) {
      for (auto& edge : edges) {
        edge.weight_up_ = kInfeasible;
        edge.weight_down_ = kInfeasible;
        edge.is_shortcut_up_ = false;
        edge.is_shortcut_down_ = false;
      }
    }

    // real edges
    for_each_edge<SearchDir>(
        way, params,
        [&](typename P::node const u, typename P::node const v, cost_t cost) {
          typename P::node source = rank_.at(u) < rank_.at(v) ? u : v;
          typename P::node target =
              rank_.at(u) > rank_.at(v) ? u : v;  // higher ranks

          for (auto& edge : up_graph_.at(source)) {  // edges to higher rank
            if (edge.to_ == target) {
              if (u == source) {
                edge.weight_up_ = std::min(edge.weight_up_, cost);
              } else {
                edge.weight_down_ = std::min(edge.weight_down_, cost);
              }
            }
          }
        });

    // shortcuts
    auto const ordered_nodes = get_ordered_nodes();  // low->high

    for (typename P::node v : ordered_nodes) {
      auto const& upward_edges = up_graph_.at(v);

      for (std::size_t i = 0; i < upward_edges.size(); ++i) {
        for (std::size_t j = i + 1; j < upward_edges.size(); ++j) {

          auto const& edge_vu = upward_edges[i];  // edge vu         u         w
          auto const& edge_vw = upward_edges[j];  // Edge vw             v

          auto u = edge_vu.to_;
          auto w = edge_vw.to_;

          typename P::node source = rank_.at(u) < rank_.at(w) ? u : w;
          typename P::node target = rank_.at(u) > rank_.at(w) ? u : w;

          cost_t cost_src_to_target;  // u v w
          cost_t cost_target_to_src;  // w v u

          if (u == source) {  // base case; fwd down up
            cost_src_to_target =
                (edge_vu.weight_down_ == kInfeasible ||
                 edge_vw.weight_up_ == kInfeasible)
                    ? kInfeasible
                    : edge_vu.weight_down_ + edge_vw.weight_up_;

            cost_target_to_src =
                (edge_vw.weight_down_ == kInfeasible ||
                 edge_vu.weight_up_ == kInfeasible)
                    ? kInfeasible
                    : edge_vw.weight_down_ + edge_vu.weight_up_;
          } else {
            cost_target_to_src =
                (edge_vu.weight_down_ == kInfeasible ||
                 edge_vw.weight_up_ == kInfeasible)
                    ? kInfeasible
                    : edge_vu.weight_down_ + edge_vw.weight_up_;

            cost_src_to_target =
                (edge_vw.weight_down_ == kInfeasible ||
                 edge_vu.weight_up_ == kInfeasible)
                    ? kInfeasible
                    : edge_vw.weight_down_ + edge_vu.weight_up_;
          }

          for (auto& shortcut : up_graph_.at(source)) {
            if (shortcut.to_ == target) {
              if (cost_src_to_target != kInfeasible &&
                  cost_src_to_target < shortcut.weight_up_) {
                shortcut.weight_up_ = cost_src_to_target;
                shortcut.inbetween_up_ = v;
                shortcut.is_shortcut_up_ = true;
              }
              if (cost_target_to_src != kInfeasible &&
                  cost_target_to_src < shortcut.weight_down_) {
                shortcut.weight_down_ = cost_target_to_src;
                shortcut.inbetween_down_ = v;
                shortcut.is_shortcut_down_ = true;
              }
            }
          }
        }
      }
    }
    for (auto& [node, edges] : up_graph_) {
      std::vector<shortcut> kept;
      for (auto const& edge : edges) {
        if (edge.weight_up_ != kInfeasible ||
            edge.weight_down_ != kInfeasible) {
          kept.push_back(edge);
        }
      }
      edges = kept;
    }
  }

  void search(auto& pq,
              auto& cost_map,
              auto const& cost_map_bwd,
              auto& pred_map,
              route& result,
              bool use_up) const {
    auto current = pq.top();
    pq.pop();

    auto const self = cost_map.find(current.node_);
    if (self != cost_map.end() &&
        current.cost_ > self->second) {  // skip if better alt
      return;
    }

    auto const current_key = cost_map_bwd.find(current.node_);
    // check for improvement
    if (current_key != cost_map_bwd.end()) {
      cost_t total_cost = current.cost_ + current_key->second;
      if (total_cost < result.cost_) {
        result.cost_ = total_cost;
        result.intersection_ = current.node_;
      }
    }

    auto const current_up_key = up_graph_.find(current.node_);
    if (current_up_key != up_graph_.end()) {
      for (auto& edge : current_up_key->second) {
        auto const edge_weight_dir =
            use_up ? edge.weight_up_ : edge.weight_down_;
        if (edge_weight_dir == kInfeasible) {
          continue;
        }
        auto const new_cost = current.cost_ + edge_weight_dir;

        auto const target = cost_map.find(edge.to_);
        if (target == cost_map.end() || target->second > new_cost) {
          cost_map[edge.to_] = new_cost;
          pred_map[edge.to_] = current.node_;
          pq.push({new_cost, edge.to_});
        }
      }
    }
  }

  route query(
      std::vector<std::pair<typename P::node, cost_t>> const& sources,
      std::vector<std::pair<typename P::node, cost_t>> const& targets) const {
    route result{};

    for (auto const& [source, source_cost] : sources) {
      for (auto const& [target, target_cost] : targets) {
        if (source == target) {
          auto const total =
              clamp_cost(static_cast<std::uint64_t>(source_cost) + target_cost);
          if (total < result.cost_) {
            result.cost_ = total;
            result.intersection_ = source;
          }
        }
      }
    }

    std::priority_queue<pq_entry, std::vector<pq_entry>, std::greater<pq_entry>>
        pq_fwd, pq_bwd;  // min pq
    hash_map<typename P::node, cost_t> fwd_cost, bwd_cost;
    hash_map<typename P::node, typename P::node> fwd_pred, bwd_pred;

    for (auto const& [source, source_cost] : sources) {
      auto const it = fwd_cost.find(source);
      if (it == fwd_cost.end() || source_cost < it->second) {
        fwd_cost[source] = source_cost;
        pq_fwd.push({source_cost, source});
      }
    }

    for (auto const& [target, target_cost] : targets) {
      auto const it = bwd_cost.find(target);
      if (it == bwd_cost.end() || target_cost < it->second) {
        bwd_cost[target] = target_cost;
        pq_bwd.push({target_cost, target});
      }
    }

    while (!pq_fwd.empty() || !pq_bwd.empty()) {
      cost_t min_fwd = pq_fwd.empty() ? kInfeasible : pq_fwd.top().cost_;
      cost_t min_bwd = pq_bwd.empty() ? kInfeasible : pq_bwd.top().cost_;

      if ((pq_fwd.empty() || min_fwd >= result.cost_) &&
          (pq_bwd.empty() || min_bwd >= result.cost_)) {
        break;
      }
      if (!pq_fwd.empty() && min_fwd <= min_bwd) {  // continue from short path
        search(pq_fwd, fwd_cost, bwd_cost, fwd_pred, result, true);
      } else if (!pq_bwd.empty()) {
        search(pq_bwd, bwd_cost, fwd_cost, bwd_pred, result, false);
      }
    }

    // build packed..
    // 1st (start->intersection) from the preds
    if (result.cost_ != kInfeasible) {

      // backwards
      auto node_to_start = result.intersection_;
      while (true) {
        result.packed_path_.push_back(node_to_start);
        auto const it = fwd_pred.find(node_to_start);
        if (it == fwd_pred.end()) {
          break;
        }
        node_to_start = it->second;
      }
      std::reverse(result.packed_path_.begin(), result.packed_path_.end());

      // 2nd (intersection -> target)
      auto node_to_target = result.intersection_;
      while (true) {
        auto const it = bwd_pred.find(node_to_target);
        if (it == bwd_pred.end()) {
          break;
        }
        node_to_target = it->second;
        result.packed_path_.push_back(node_to_target);
      }
      // result (start-intersection->target)
    }
    return result;
  }

  void preprocess(ways const& way,
                  direction const dir,
                  typename P::parameters const& params) {
    if (dir == direction::kForward) {
      build_topology<direction::kForward>(way);
      customise<direction::kForward>(way, params);
    } else {
      build_topology<direction::kBackward>(way);
      customise<direction::kBackward>(way, params);
    }
  }
};
}  // namespace osr
