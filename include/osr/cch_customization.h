#pragma once

#include <iostream>

#include <utility>

#include "osr/cch_preprocessing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/with_profile.h"
#include "osr/shortcut.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace cch {

struct customization {

  struct way_data {
    osr::way_idx_t way_;
    osr::direction dir_;
    std::uint16_t node_in_way_idx_;
  };

  customization(cista::wrapped<osr::ways::routing>& r) : r_{r} {}

  void get_cch_edges(osr::search_profile const& profile,
                     osr::profile_parameters const& params) {
    return with_valid_cch_profile(profile, [&]<osr::Profile P>(P&&) {
      auto const& pp = std::get<typename P::parameters>(params);
      return get_cch_edges<P>(pp);
    });
  }

  template <bool WithRestrictions, bool IsBus>
  void customize_shortcuts(osr::search_profile const& profile,
                           osr::profile_parameters const& params) {
    return with_valid_cch_profile(profile, [&]<osr::Profile P>(P&&) {
      auto const& pp = std::get<typename P::parameters>(params);
      return customize_shortcuts<P, WithRestrictions, IsBus>(pp);
    });
  }

  // intiliaze the shortcuts of length 1 (the real edges in this function)
  // allocate the necessary memory within the routing struct here:
  template <osr::Profile P>
  void get_cch_edges(typename P::parameters const& params) {
    r_->cch_cost_up_.resize(r_->contraction_order_.size());
    r_->cch_cost_down_.resize(r_->contraction_order_.size());
    r_->cch_cost_self_.resize(r_->contraction_order_.size());
    r_->cch_sc_up_.resize(r_->contraction_order_.size());
    r_->cch_sc_down_.resize(r_->contraction_order_.size());
    r_->cch_sc_self_.resize(r_->contraction_order_.size());

    for (auto const [rank64, node] : utl::enumerate(r_->contraction_order_)) {
      auto const rank = static_cast<std::uint32_t>(rank64);
      auto const target_size =
          static_cast<std::uint32_t>(r_->sc_targets_[rank].size());
      auto const self_size =
          static_cast<std::uint32_t>(r_->node_ways_[node].size());
      r_->cch_cost_up_[rank].resize(target_size, osr::kInfeasible);
      r_->cch_cost_down_[rank].resize(target_size, osr::kInfeasible);
      r_->cch_cost_self_[rank].resize(self_size, osr::kInfeasible);
      r_->cch_sc_up_[rank].resize(target_size, packed_shortcut::invalid());
      r_->cch_sc_down_[rank].resize(target_size, packed_shortcut::invalid());
      r_->cch_sc_self_[rank].resize(self_size, packed_shortcut::invalid());

      auto const node_cost = P::node_cost(params, r_->node_properties_[node]);
      if (node_cost == osr::kInfeasible) {
        continue;
      }

      // the structure of the for loop is inspired by the
      // for_each_adjacent_node function profiles/common.h
      for (auto const [way, idx] :
           utl::zip(r_->node_ways_[node], r_->node_in_way_idx_[node])) {
        auto const get_edge = [&](osr::direction const dir,
                                  std::uint16_t const from,
                                  std::uint16_t const to) {
          // check importance and accessibility
          auto const neighbor = r_->way_nodes_[way][to];
          auto const neighbor_p = r_->node_properties_[neighbor];
          if (r_->node_importance_[neighbor] <= rank ||
              !neighbor_p.is_car_accessible_) {
            return;
          }

          auto const wp = r_->way_properties_[way];
          auto const target_idx = r_->get_target_idx(node, neighbor);
          auto const dist = r_->get_way_node_distance(way, std::min(from, to));
          auto const node_way_pos = r_->get_way_pos(node, way, from);
          auto const neighbor_way_pos = r_->get_way_pos(neighbor, way, to);

          // check way cost up
          if (P::way_cost(params, wp, dir, 0U) != osr::kInfeasible &&
              P::node_cost(params, neighbor_p) != osr::kInfeasible) {
            auto const new_way_cost_up = P::way_cost(params, wp, dir, dist) +
                                         P::node_cost(params, neighbor_p);
            if (new_way_cost_up < r_->cch_cost_up_[rank][target_idx]) {
              r_->cch_cost_up_[rank][target_idx] = new_way_cost_up;
              r_->cch_sc_up_[rank][target_idx] = cch::packed_shortcut{
                  .entry_node_ = cch::target_node{node, node_way_pos, dir},
                  .exit_node_ =
                      cch::target_node{neighbor, neighbor_way_pos, dir},
                  .down_ = 0U,
                  .up_ = 0U,
                  .via_rank_ = 0U,
                  .u_turn_penalty_ = osr::cost_t{0U}};
            }
          }

          // check way cost down
          if (P::way_cost(params, wp, osr::opposite(dir), 0U) !=
                  osr::kInfeasible &&
              node_cost != osr::kInfeasible) {
            auto const new_way_cost_down =
                P::way_cost(params, wp, osr::opposite(dir), dist) + node_cost;
            if (new_way_cost_down < r_->cch_cost_down_[rank][target_idx]) {
              r_->cch_cost_down_[rank][target_idx] = new_way_cost_down;
              r_->cch_sc_down_[rank][target_idx] = cch::packed_shortcut{
                  .entry_node_ = cch::target_node{neighbor, neighbor_way_pos,
                                                  osr::opposite(dir)},
                  .exit_node_ =
                      cch::target_node{node, node_way_pos, osr::opposite(dir)},
                  .down_ = 0U,
                  .up_ = 0U,
                  .via_rank_ = 0U,
                  .u_turn_penalty_ = osr::cost_t{0}};
            }
          }
        };

        if (idx != 0U) {
          get_edge(osr::direction::kBackward, idx, idx - 1);
        }
        if (idx != r_->way_nodes_[way].size() - 1U) {
          get_edge(osr::direction::kForward, idx, idx + 1);
        }
      }
    }
  }

  // perform the basic customization of the shortcuts according to the survey.
  // To handle turn restrictions, add shortcuts as loop from the upper neighbor
  // down to the current node and back to the neighbor
  template <osr::Profile P, bool WithRestrictions, bool IsBus>
  void customize_shortcuts(typename P::parameters const& params) {
    for (auto const [rank64, node] : utl::enumerate(r_->contraction_order_)) {
      auto const rank = static_cast<std::uint32_t>(rank64);
      auto const& neighbors = r_->sc_targets_[rank];
      utl::verify(neighbors.size() == r_->cch_sc_up_[rank].size() &&
                      neighbors.size() == r_->cch_sc_down_[rank].size() &&
                      neighbors.size() == r_->cch_cost_up_[rank].size() &&
                      neighbors.size() == r_->cch_cost_down_[rank].size(),
                  "[CCH Customization] Unequal array sizes");
      if (neighbors.empty() || !r_->node_properties_[node].is_car_accessible_) {
        continue;
      }
      // shortcut goes entry -> node -> target
      for (auto const [n_idx64, entry] : utl::enumerate(neighbors)) {
        auto const n_idx = static_cast<std::uint32_t>(n_idx64);
        auto const& n_rank = r_->node_importance_[entry];
        auto const& targets = r_->sc_targets_[n_rank];
        auto const& node_to_entry_cost = r_->cch_cost_up_[rank][n_idx];
        auto const& entry_to_node_cost = r_->cch_cost_down_[rank][n_idx];

        // add phantom shortcut from neighbor to neighbor: (indexed by way idx
        // of node)
        auto const& to_entry = r_->cch_sc_up_[rank][n_idx];
        auto const& from_entry = r_->cch_sc_down_[rank][n_idx];
        auto const& penalty = get_penalty<P, WithRestrictions, IsBus>(
            params, node, r_->cch_sc_down_[rank][n_idx],
            r_->cch_sc_up_[rank][n_idx]);
        auto const& sc_idx =
            static_cast<std::uint32_t>(from_entry.entry_node_.way_);
        if (to_entry.is_valid() && from_entry.is_valid()) {
          if (r_->cch_true_edge(to_entry) && r_->cch_true_edge(from_entry) &&
              combineable(r_->cch_cost_self_[n_rank][sc_idx],
                          node_to_entry_cost, entry_to_node_cost, penalty)) {
            r_->cch_cost_self_[n_rank][sc_idx] =
                node_to_entry_cost + entry_to_node_cost + penalty;
            combine_shortcuts(r_->cch_sc_self_[n_rank][sc_idx], rank, n_idx,
                              n_idx, penalty);
          }
        }

        // add regular shortcut (indexed by index of target in sc_targets)
        for (std::uint32_t t_idx = n_idx + 1; t_idx < neighbors.size();
             ++t_idx) {
          auto const& target = neighbors[t_idx];
          utl::verify(
              r_->node_importance_[target] > r_->node_importance_[entry],
              "[CUSTOMIZATION] Expected higher rank from entry to target");
          auto const t_n_idx = find_target(targets, target);

          if (t_n_idx == targets.size()) {
            continue;
          }

          // check for shortcut up
          auto const& entry_to_target_cost = r_->cch_cost_up_[n_rank][t_n_idx];
          auto u_turn_penalty_up = osr::kInfeasible;
          if (entry_to_node_cost != osr::kInfeasible &&
              r_->cch_cost_up_[rank][t_idx] != osr::kInfeasible) {
            u_turn_penalty_up = get_penalty<P, WithRestrictions, IsBus>(
                params, node, r_->cch_sc_down_[rank][n_idx],
                r_->cch_sc_up_[rank][t_idx]);
          }

          if (combineable(entry_to_target_cost, r_->cch_cost_up_[rank][t_idx],
                          entry_to_node_cost, u_turn_penalty_up)) {
            r_->cch_cost_up_[n_rank][t_n_idx] = entry_to_node_cost +
                                                u_turn_penalty_up +
                                                r_->cch_cost_up_[rank][t_idx];
            auto& shortcut_up = r_->cch_sc_up_[n_rank][t_n_idx];
            combine_shortcuts(shortcut_up, rank, n_idx, t_idx,
                              u_turn_penalty_up);
          }

          // check for shortcut down
          auto const& target_to_entry_cost =
              r_->cch_cost_down_[n_rank][t_n_idx];
          auto u_turn_penalty_down = osr::kInfeasible;
          if (node_to_entry_cost != osr::kInfeasible &&
              r_->cch_cost_down_[rank][t_idx] != osr::kInfeasible) {
            u_turn_penalty_down = get_penalty<P, WithRestrictions, IsBus>(
                params, node, r_->cch_sc_down_[rank][t_idx],
                r_->cch_sc_up_[rank][n_idx]);
          }

          if (combineable(target_to_entry_cost, node_to_entry_cost,
                          r_->cch_cost_down_[rank][t_idx],
                          u_turn_penalty_down)) {
            r_->cch_cost_down_[n_rank][t_n_idx] =
                r_->cch_cost_down_[rank][t_idx] + u_turn_penalty_down +
                node_to_entry_cost;
            auto& shortcut_down = r_->cch_sc_down_[n_rank][t_n_idx];
            combine_shortcuts(shortcut_down, rank, t_idx, n_idx,
                              u_turn_penalty_down);
          }
        }
      }
    }
  }

  // check if two shortcuts, merged to one shortcut, have to add a uturn penalty
  // and check for turn restrictions:
  template <osr::Profile P, bool WithRestrictions, bool IsBus>
  osr::cost_t get_penalty(typename P::parameters const& params,
                          osr::node_idx_t const& n,
                          packed_shortcut const& down,
                          packed_shortcut const& up) {
    auto const& exit_down = down.exit_node_;
    auto const& entry_up = up.entry_node_;

    utl::verify(
        (exit_down.n_ == n || exit_down.n_ == osr::node_idx_t::invalid()) &&
            (entry_up.n_ == n || entry_up.n_ == osr::node_idx_t::invalid()),
        "[GET_PENALTY] Expected same meetpoint at {} but got {} and {}", n,
        exit_down.n_, entry_up.n_);

    if (exit_down.n_ != entry_up.n_) {
      return osr::kInfeasible;
    }

    if constexpr (WithRestrictions) {
      if (r_->is_restricted<osr::direction::kForward, IsBus>(n, exit_down.way_,
                                                             entry_up.way_)) {
        return osr::kInfeasible;
      }
    }

    if (exit_down.way_ == entry_up.way_ &&
        exit_down.dir_ == osr::opposite(entry_up.dir_)) {
      return params.uturn_penalty_;
    } else {
      return osr::cost_t{0U};
    }
  }

  // check if shortcuts are combineable based on their cost
  bool combineable(osr::cost_t const& old_cost,
                   osr::cost_t const& via_to_target_cost,
                   osr::cost_t const& entry_to_via_cost,
                   osr::cost_t const& penalty) {
    if (penalty == osr::kInfeasible || via_to_target_cost == osr::kInfeasible ||
        entry_to_via_cost == osr::kInfeasible) {
      return false;
    }

    auto const new_cost = entry_to_via_cost + penalty + via_to_target_cost;
    if (old_cost <= new_cost || new_cost == osr::kInfeasible) {
      return false;
    }

    return true;
  }

  void combine_shortcuts(packed_shortcut& new_shortcut,
                         std::uint32_t const via_rank,
                         std::uint32_t const entry_idx,
                         std::uint32_t const target_idx,
                         osr::cost_t const& penalty) {
    auto const& new_entry = r_->cch_sc_down_[via_rank][entry_idx].entry_node_;
    auto const& new_exit = r_->cch_sc_up_[via_rank][target_idx].exit_node_;

    utl::verify(new_entry.is_valid() && new_exit.is_valid(),
                "[CCH Shortcut Combination] Failed to combine shortcuts due to "
                "invalid target nodes");
    new_shortcut.entry_node_ = new_entry;
    new_shortcut.exit_node_ = new_exit;
    new_shortcut.down_ = entry_idx;
    new_shortcut.up_ = target_idx;
    new_shortcut.via_rank_ = via_rank;
    new_shortcut.u_turn_penalty_ = penalty;
  }

  // find the index of the target for a shortcut in the list of upper neighbors
  // (sc_targets in the routing struct)
  std::uint32_t find_target(osr::vec<osr::node_idx_t> const& targets,
                            osr::node_idx_t const& t) {
    for (auto const [i, n] : utl::enumerate(targets)) {
      if (n == t) {
        return static_cast<std::uint32_t>(i);
      }
    }
    return targets.size();
  }

  cista::wrapped<osr::ways::routing>& r_;
};
}  // namespace cch