#pragma once

#include <vector>

#include "utl/enumerate.h"
#include "utl/erase_duplicates.h"
#include "utl/verify.h"
#include "utl/zip.h"

#include "osr/cch_preprocessing/extended_type.h"
#include "osr/cch_preprocessing/node_ordering.h"
#include "osr/routing/profile.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr::cch_preprocessing {

using graph = std::vector<std::vector<std::uint32_t>>;

template <Profile P>
struct customized_cost_stored;

struct elimination_tree {
  static constexpr auto const kDoFastContraction = true;

  static void normal_contraction(graph& adj, std::vector<std::uint32_t>& res);
  static void fast_contraction(graph const& adj,
                               std::vector<std::uint32_t>& res);
  static elimination_tree compute(node_ordering const& ordering, ways const& w);

  template <std::size_t NMaxTypes>
  friend constexpr auto static_type_hash(
      elimination_tree const*, cista::hash_data<NMaxTypes> h) noexcept {
    using cista::static_type_hash;
    h = h.combine(cista::hash("elimination tree v1.0"));
    return h;
  }

  static cista::wrapped<elimination_tree> read(
      std::filesystem::path const& path) {
    return cista::read<elimination_tree>(path / "elimination_tree.bin");
  }

  void write(std::filesystem::path const& path) const {
    return cista::write(path / "elimination_tree.bin", *this);
  }

  vec_map<node_idx_t, node_idx_t> tree_;
};

}  // namespace osr::cch_preprocessing