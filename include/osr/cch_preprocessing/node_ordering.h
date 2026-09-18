#pragma once

#include <filesystem>
#include <random>

#include "osr/cch_preprocessing/extended_type.h"
#include "osr/types.h"

#include "utl/enumerate.h"
#include "utl/helpers/algorithm.h"
#include "utl/verify.h"

namespace osr::cch_preprocessing {
struct node_ordering {

  inline std::uint32_t size() const { return old_to_new_.size(); }

  node_idx_t const get_ordering(node_idx_t const node) const {
    utl::verify(node < old_to_new_.size(),
                "node index {} out of bounds, max {}", node,
                old_to_new_.size());
    return old_to_new_.at(node);
  }

  node_idx_t const get_node(node_idx_t const ordering) const {
    utl::verify(ordering < new_to_old_.size(),
                "ordering index {} out of bounds, max {}", ordering,
                new_to_old_.size());
    return new_to_old_.at(ordering);
  }

  template <bool isNewOrd, typename Fn>
  void for_each_node(Fn&& fn) const {
    if constexpr (isNewOrd) {
      for (auto const [new_idx, old_idx] : utl::enumerate(new_to_old_)) {
        fn(static_cast<node_idx_t>(new_idx), old_idx);
      }
    } else {
      for (auto const [old_idx, new_idx] : utl::enumerate(old_to_new_)) {
        fn(static_cast<node_idx_t>(old_idx), new_idx);
      }
    }
  }

  template <std::size_t NMaxTypes>
  friend constexpr auto static_type_hash(
      node_ordering const*, cista::hash_data<NMaxTypes> h) noexcept {
    using cista::static_type_hash;
    h = h.combine(cista::hash("node_ordering v1.0"));
    return h;
  }

  static cista::wrapped<node_ordering> read(std::filesystem::path const& path) {
    return cista::read<node_ordering>(path / "node_ordering.bin");
  }

  void write(std::filesystem::path const& path) const {
    return cista::write(path / "node_ordering.bin", *this);
  }

  template <HasNodeImportance T>
  static node_ordering import(T const& t) {
    node_ordering ordering;
    ordering.old_to_new_.resize(t.node_importance_.size());
    ordering.new_to_old_.resize(t.node_importance_.size());
    vec<std::pair<std::uint32_t, node_idx_t>> nodes;
    for (auto const& [node, importance] : utl::enumerate(t.node_importance_)) {
      nodes.emplace_back(importance, static_cast<node_idx_t>(node));
    }
    utl::sort(nodes);
    for (std::uint32_t i = 0U; i < nodes.size(); ++i) {
      auto const node = nodes[i].second;
      ordering.old_to_new_[node] = static_cast<node_idx_t>(i);
      ordering.new_to_old_[static_cast<node_idx_t>(i)] = node;
    }
    return ordering;
  }

  static node_ordering randomize(std::uint32_t num_nodes, std::uint32_t seed);

  vec_map<node_idx_t, node_idx_t> old_to_new_;
  vec_map<node_idx_t, node_idx_t> new_to_old_;
};
}  // namespace osr::cch_preprocessing