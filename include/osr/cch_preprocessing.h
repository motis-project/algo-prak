#pragma once

#include <algorithm>
#include <numeric>
#include <utility>

#include "cista/containers/vector.h"
#include "cista/io.h"
#include "cista/strong.h"
#include "utl/enumerate.h"

#include "osr/routing/parameters.h"
#include "osr/routing/profiles/car.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace cch {

struct contraction {
  explicit contraction(cista::wrapped<osr::ways::routing>&);

  // helper functions:
  void build_contraction_order() {
    r_->contraction_order_.resize(r_->node_importance_.size());
    for (auto const [i, rank] : utl::enumerate(r_->node_importance_)) {
      r_->contraction_order_[rank] = osr::node_idx_t{i};
    }
  }

  // sort the neighbors of a node by rank
  void sort_and_filter_neighbors(std::uint32_t const& rank) {
    if (r_->sc_targets_[rank].size() < 2) {
      return;
    }
    // source for sorting condition -> searched a basic cpp feature here:
    // https://stackoverflow.com/questions/23816797/how-does-stdsort-work-for-list-of-pairs#23817006
    // (11.06.2026)
    auto sorting_condition = [this](osr::node_idx_t const lhs,
                                    osr::node_idx_t const rhs) {
      return r_->node_importance_[lhs] < r_->node_importance_[rhs];
    };
    // sort the given neighbors
    std::sort(r_->sc_targets_[rank].begin(), r_->sc_targets_[rank].end(),
              sorting_condition);
    // filter duplicates
    auto last_s =
        std::unique(r_->sc_targets_[rank].begin(), r_->sc_targets_[rank].end());
    r_->sc_targets_[rank].erase(last_s, r_->sc_targets_[rank].end());

    for (std::uint32_t i = 0; i < (r_->sc_targets_[rank].size() - 1); ++i) {
      utl::verify(r_->node_importance_[r_->sc_targets_[rank][i]] <
                      r_->node_importance_[r_->sc_targets_[rank][i + 1]],
                  "Neighbors are sorted incorrectly");
    }
    utl::verify(
        rank < r_->node_importance_[r_->sc_targets_[rank][0]],
        "Got false lowest higher neighbor of rank {} at next with rank {}",
        r_->node_importance_[r_->sc_targets_[rank][0]], rank);
  }

  // main functions:
  void init_neighborhoods();
  void contract_nodes();

  cista::wrapped<osr::ways::routing>& r_;
};

}  // namespace cch