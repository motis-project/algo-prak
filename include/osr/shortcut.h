#pragma once

#include <numeric>

#include "utl/verify.h"

#include "osr/types.h"
#include "osr/ways.h"

namespace cch {

struct target_node {
  friend bool operator==(target_node, target_node) = default;
  static constexpr target_node invalid() noexcept {
    return target_node{.n_ = osr::node_idx_t::invalid(),
                       .way_ = osr::way_pos_t{0U},
                       .dir_ = osr::direction::kForward};
  }

  bool is_valid() const { return n_ != osr::node_idx_t::invalid(); }

  osr::node_idx_t n_;
  osr::way_pos_t way_;
  osr::direction dir_;
};

// a packed shortcut containts the entry and exit node
// for recursive packing, the rank other node they are
// combined and also the the index of the merged shortcuts
// for up and down.
// Example: Down shortcut is at routing->cch_sc_down_[via_rank_][down_]
struct packed_shortcut {
  static constexpr packed_shortcut invalid() noexcept {
    return packed_shortcut{.entry_node_ = target_node::invalid(),
                           .exit_node_ = target_node::invalid(),
                           .down_ = 0U,
                           .up_ = 0U,
                           .via_rank_ = 0U,
                           .u_turn_penalty_ = osr::kInfeasible};
  }

  bool is_valid() const {
    return entry_node_.is_valid() && exit_node_.is_valid() &&
           u_turn_penalty_ != osr::kInfeasible;
  }

  // The first approach was to use std::optional pointers to link
  // the recursive shortcuts. After running into multiple cista errors
  // I used [AI] to let me explain the handling of pointers in cista.
  // It recommended the use of cista::offset::unique_ptr<>(...).
  // I developed an approach but run into malloc errors.
  // Then I used indices instead.
  target_node entry_node_;
  target_node exit_node_;
  std::uint32_t down_;
  std::uint32_t up_;
  std::uint32_t via_rank_;
  osr::cost_t u_turn_penalty_;
};

}  // namespace cch