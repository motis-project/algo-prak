#include "osr/cch_preprocessing.h"

#include <iostream>
#include <vector>

#include "utl/verify.h"

#include "osr/ways.h"

cch::contraction::contraction(cista::wrapped<osr::ways::routing>& r) : r_{r} {}

// calculate the neighborhood of all nodes in the graph
void cch::contraction::init_neighborhoods() {
  if (r_->contraction_order_.empty()) {
    return;
  }
  r_->sc_targets_.resize(r_->contraction_order_.size());
  for (auto const [rank64, node] : utl::enumerate(r_->contraction_order_)) {
    auto const rank = static_cast<std::uint32_t>(rank64);
    utl::verify(rank == r_->node_importance_[node],
                "Expected Node {} with rank {} but node came at rank {}", node,
                r_->node_importance_[node], rank);

    auto const& in_ways = r_->node_ways_[node];
    auto const& idx_in_ways = r_->node_in_way_idx_[node];
    if (in_ways.empty() && idx_in_ways.empty()) {
      continue;
    }

    for (auto const [idx, way] : utl::zip(idx_in_ways, in_ways)) {
      // add neighbors in forward direction with higher rank
      if (idx > 0) {
        auto const pred = r_->way_nodes_[way][idx - 1];
        if (rank < r_->node_importance_[pred]) {
          r_->sc_targets_[rank].push_back(pred);
        }
      }
      // add neighbors in backward direction with higher rank
      if (idx < (r_->way_nodes_[way].size() - 1)) {
        auto const succ = r_->way_nodes_[way][idx + 1];
        if (rank < r_->node_importance_[succ]) {
          r_->sc_targets_[rank].push_back(succ);
        }
      }
    }
  }
}

// contract the nodes sorted by rank
void cch::contraction::contract_nodes() {
  if (r_->sc_targets_.empty()) {
    return;
  }

  for (std::uint32_t rank = 0; rank < r_->contraction_order_.size(); ++rank) {
    if (r_->sc_targets_[rank].size() == 0) {
      continue;
    }
    sort_and_filter_neighbors(rank);
    auto const& neighbors = r_->sc_targets_[rank];
    auto const& next_rank = r_->node_importance_[neighbors[0]];
    for (auto const& n : neighbors) {
      if (r_->node_importance_[n] > next_rank) {
        r_->sc_targets_[next_rank].push_back(n);
      }
    }
  }
}