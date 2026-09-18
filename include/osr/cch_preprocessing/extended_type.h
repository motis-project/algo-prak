#pragma once

#include "osr/types.h"

namespace osr::cch_preprocessing {

template <typename T>
concept HasNodeImportance = requires(T t, node_idx_t idx) {
  { t.node_importance_[idx] } noexcept -> std::same_as<std::uint32_t&>;
};

using ext_edge_idx_t = way_idx_t;

struct ext_node {

  static auto to_ext_node(std::uint32_t const primary_idx,
                          std::uint16_t const sub_idx) {
    return ext_node{.primary_idx_ = static_cast<node_idx_t>(primary_idx),
                    .sub_idx_ = sub_idx};
  }

  static constexpr auto invalid() {
    return ext_node{.primary_idx_ = node_idx_t::invalid(),
                    .sub_idx_ = std::numeric_limits<std::uint16_t>::max()};
  }

  friend constexpr auto operator<=>(ext_node const& a,
                                    ext_node const& b) noexcept {
    return std::tie(a.primary_idx_, a.sub_idx_) <=>
           std::tie(b.primary_idx_, b.sub_idx_);
  }

  friend constexpr auto operator==(ext_node const& a,
                                   ext_node const& b) noexcept {
    return std::tie(a.primary_idx_, a.sub_idx_) ==
           std::tie(b.primary_idx_, b.sub_idx_);
  }

  friend constexpr auto operator!=(ext_node const& a,
                                   ext_node const& b) noexcept {
    return std::tie(a.primary_idx_, a.sub_idx_) !=
           std::tie(b.primary_idx_, b.sub_idx_);
  }

  node_idx_t const first() const { return primary_idx_; }
  std::uint16_t second() const { return sub_idx_; }

  node_idx_t primary_idx_;
  std::uint16_t sub_idx_;
};

struct ext_edge {
  template <std::size_t NMaxTypes>
  friend constexpr auto static_type_hash(
      ext_edge const*, cista::hash_data<NMaxTypes> h) noexcept {
    using cista::static_type_hash;
    h = h.combine(cista::hash("ext_edge v1.0"));
    return h;
  }

  inline bool is_original() const {
    return traceback_ ==
           pair{ext_edge_idx_t::invalid(), ext_edge_idx_t::invalid()};
  }

  void update(cost_t const new_cost,
              ext_edge_idx_t const first_half,
              ext_edge_idx_t const second_half) {
    if (new_cost < cost_) {
      cost_ = new_cost;
      traceback_ = pair{first_half, second_half};
    }
  }

  void update(cost_t const new_cost) {
    if (new_cost < cost_) {
      cost_ = new_cost;
      traceback_ = pair{ext_edge_idx_t::invalid(), ext_edge_idx_t::invalid()};
    }
  }

  ext_node from_, to_;
  cost_t cost_ = kInfeasible;
  pair<ext_edge_idx_t, ext_edge_idx_t> traceback_ =
      pair{ext_edge_idx_t::invalid(), ext_edge_idx_t::invalid()};
};

struct node_entry {
  static constexpr node_entry const invalid() {
    return node_entry{.cost_ = kInfeasible, .pred_ = ext_edge_idx_t::invalid()};
  }

  ext_edge_idx_t const pred() const { return pred_; }

  void update(cost_t const new_cost, ext_edge_idx_t const pred) {
    if (new_cost < cost_) {
      cost_ = new_cost;
      pred_ = pred;
    }
  }

  friend constexpr auto operator<=>(node_entry const& a,
                                    node_entry const& b) noexcept {
    return std::tie(a.cost_, a.pred_) <=> std::tie(b.cost_, b.pred_);
  }

  friend constexpr auto operator==(node_entry const& a,
                                   node_entry const& b) noexcept {
    return std::tie(a.cost_, a.pred_) == std::tie(b.cost_, b.pred_);
  }

  friend constexpr auto operator!=(node_entry const& a,
                                   node_entry const& b) noexcept {
    return std::tie(a.cost_, a.pred_) != std::tie(b.cost_, b.pred_);
  }

  cost_t cost_;
  ext_edge_idx_t pred_;
};

constexpr std::string to_string(ext_node idx) {
  return fmt::format("({}, {})", idx.first(), idx.second());
}

constexpr std::string to_string(ext_edge_idx_t idx) {
  return fmt::format("({})", idx);
}

}  // namespace osr::cch_preprocessing