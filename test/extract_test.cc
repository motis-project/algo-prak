#include "gtest/gtest.h"

#include <algorithm>
#include <filesystem>
#include <iostream>

#include "cista/mmap.h"

#include "osr/cch_customization.h"
#include "osr/cch_preprocessing.h"
#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/route.h"
#include "osr/shortcut.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(extract, string_cache) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/map.osm", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  // 140186757 and 519430215 have the same name=Pankratiusstraße
  const auto pankratius1 = w.find_way(osm_way_idx_t{140186757});
  const auto pankratius2 = w.find_way(osm_way_idx_t{519430215});
  ASSERT_TRUE(pankratius1.has_value());
  ASSERT_TRUE(pankratius2.has_value());

  const auto name_idx1 = w.way_names_[pankratius1.value()];
  const auto name_idx2 = w.way_names_[pankratius2.value()];
  ASSERT_EQ(name_idx1, name_idx2);
}

TEST(extract, bus_only_on_highway) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/luisenplatz-darmstadt.osm.pbf", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  auto const luisenplatz_outer = w.find_way(osm_way_idx_t{53341306});
  ASSERT_TRUE(luisenplatz_outer.has_value());

  auto const& wp = w.r_->way_properties_[luisenplatz_outer.value()];
  ASSERT_FALSE(wp.is_bus_accessible());
  ASSERT_TRUE(wp.is_foot_accessible());
}

TEST(extract, contraction_order) {
  auto const data_dir = "test/hamburg.osm.pbf";
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  if (!fs::exists(data_dir)) {
    GTEST_SKIP() << data_dir << " not found";
  }

  extract(false, data_dir, p, {}, true);

  auto w = ways{p, cista::mmap::protection::READ};
  auto mip = cch::contraction{w.r_};
  mip.build_contraction_order();

  bool eq = true;
  for (auto const [rank, node] : utl::enumerate(w.r_->contraction_order_)) {
    eq = eq &&
         (static_cast<std::uint32_t>(rank) == w.r_->node_importance_[node]);
  }

  ASSERT_TRUE(eq);
}

TEST(extract, test_packed_and_unpacked_costs) {
  auto const data_dir = "test/darmstadt-bismarckstr.osm.pbf";
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  if (!fs::exists(data_dir)) {
    GTEST_SKIP() << data_dir << " not found";
  }

  extract(false, data_dir, p, {}, true);
  auto w = ways{p, cista::mmap::protection::READ};

  for (auto const [rank64, node] : utl::enumerate(w.r_->contraction_order_)) {
    auto const rank = static_cast<std::uint32_t>(rank64);
    for (auto const [t_idx64, target] :
         utl::enumerate(w.r_->sc_targets_[rank])) {
      auto const t_idx = static_cast<std::uint32_t>(t_idx64);
      auto const& shortcut = w.r_->cch_sc_up_[rank][t_idx];
      auto const& expected_cost = w.r_->cch_cost_up_[rank][t_idx];
      if (expected_cost == kInfeasible) {
        continue;
      }

      auto const path_up =
          w.r_->unpack_shortcut<direction::kForward, true>(shortcut);
      auto single_costs = w.r_->get_edge_cost<true>(
          shortcut.entry_node_.n_, shortcut.entry_node_.way_,
          shortcut.entry_node_.dir_, path_up[0].n_, cost_t{120U});

      for (std::uint32_t i = 1; i < path_up.size(); ++i) {
        single_costs += w.r_->get_edge_cost<true>(
            path_up[i - 1].n_, path_up[i - 1].way_, path_up[i - 1].dir_,
            path_up[i].n_, cost_t{120U});
      }
      ASSERT_EQ(single_costs, expected_cost);
    }

    for (auto const [t_idx64, target] :
         utl::enumerate(w.r_->sc_targets_[rank])) {
      auto const t_idx = static_cast<std::uint32_t>(t_idx64);
      auto const& shortcut = w.r_->cch_sc_down_[rank][t_idx];
      auto const& expected_cost = w.r_->cch_cost_down_[rank][t_idx];
      if (expected_cost == kInfeasible) {
        continue;
      }

      auto path_down =
          w.r_->unpack_shortcut<direction::kBackward, false>(shortcut);
      std::reverse(path_down.begin(), path_down.end());
      auto single_costs = w.r_->get_edge_cost<false>(
          shortcut.exit_node_.n_, shortcut.exit_node_.way_,
          shortcut.exit_node_.dir_, path_down[0].n_, cost_t{120U});

      for (std::uint32_t i = 1; i < path_down.size(); ++i) {
        single_costs += w.r_->get_edge_cost<false>(
            path_down[i - 1].n_, path_down[i - 1].way_, path_down[i - 1].dir_,
            path_down[i].n_, cost_t{120U});
      }
      ASSERT_EQ(single_costs, expected_cost);
    }
  }
}

TEST(extract, pack_shortcuts) {
  auto const data_dir = "test/darmstadt-bismarckstr.osm.pbf";
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  if (!fs::exists(data_dir)) {
    GTEST_SKIP() << data_dir << " not found";
  }

  extract(false, data_dir, p, {}, true);
  auto w = ways{p, cista::mmap::protection::READ};
  for (auto const [rank64, node] : utl::enumerate(w.r_->contraction_order_)) {
    auto const rank = static_cast<std::uint32_t>(rank64);
    for (auto [t_idx64, target] : utl::enumerate(w.r_->sc_targets_[rank])) {
      auto const t_idx = static_cast<std::uint32_t>(t_idx64);
      auto const& cost_up = w.r_->cch_cost_up_[rank][t_idx];
      auto const& cost_down = w.r_->cch_cost_down_[rank][t_idx];
      auto const& sc_up = w.r_->cch_sc_up_[rank][t_idx];
      auto const& sc_down = w.r_->cch_sc_down_[rank][t_idx];

      ASSERT_TRUE(w.r_->node_importance_[target] > rank);

      ASSERT_TRUE((!sc_up.is_valid() && cost_up == osr::kInfeasible) ||
                  (sc_up.is_valid() && cost_up != osr::kInfeasible));

      ASSERT_TRUE((!sc_down.is_valid() && cost_down == osr::kInfeasible) ||
                  (sc_down.is_valid() && cost_down != osr::kInfeasible));

      ASSERT_TRUE(
          (sc_up.entry_node_.n_ == node && sc_up.exit_node_.n_ == target) ||
          (cost_up == osr::kInfeasible &&
           sc_up.entry_node_.n_ == osr::node_idx_t::invalid() &&
           sc_up.exit_node_.n_ == osr::node_idx_t::invalid()));

      ASSERT_TRUE(
          (sc_down.entry_node_.n_ == target && sc_down.exit_node_.n_ == node) ||
          (cost_down == osr::kInfeasible &&
           sc_down.entry_node_.n_ == osr::node_idx_t::invalid() &&
           sc_down.exit_node_.n_ == osr::node_idx_t::invalid()));
    }

    for (auto [way_pos64, way] : utl::enumerate(w.r_->node_ways_[node])) {
      auto const way_pos = static_cast<std::uint32_t>(way_pos64);

      ASSERT_TRUE(
          w.r_->node_ways_[node].size() == w.r_->cch_sc_self_[rank].size() &&
          w.r_->node_ways_[node].size() == w.r_->cch_cost_self_[rank].size());

      auto const& self_cost = w.r_->cch_cost_self_[rank][way_pos];
      auto const& self_sc = w.r_->cch_sc_self_[rank][way_pos];

      ASSERT_TRUE(
          (self_sc.entry_node_.n_ == node && self_sc.exit_node_.n_ == node) ||
          (self_sc.entry_node_.n_ == osr::node_idx_t::invalid() &&
           self_sc.exit_node_.n_ == osr::node_idx_t::invalid()));
    }
  }
}

TEST(extract, unpack_bigger_shortcut_forward) {
  auto const data_dir = "test/darmstadt-bismarckstr.osm.pbf";
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  if (!fs::exists(data_dir)) {
    GTEST_SKIP() << data_dir << " not found";
  }

  extract(false, data_dir, p, {}, true);
  auto w = ways{p, cista::mmap::protection::READ};

  std::uint32_t const via_node_1 = 83;
  std::uint32_t const via_node_2 = 134;
  std::uint32_t const via_node_3 = 137;
  std::uint32_t const via_node_4 = 133;
  std::uint32_t const via_node_5 = 168;

  if (w.r_->contraction_order_[via_node_1] != osr::node_idx_t{222} ||
      w.r_->contraction_order_[via_node_2] != osr::node_idx_t{226} ||
      w.r_->contraction_order_[via_node_3] != osr::node_idx_t{199} ||
      w.r_->contraction_order_[via_node_4] != osr::node_idx_t{214} ||
      w.r_->contraction_order_[via_node_5] != osr::node_idx_t{188}) {
    GTEST_SKIP() << data_dir << " with invalid version";
  }

  auto const& sc_1_up =
      w.r_->cch_sc_up_[via_node_1][static_cast<std::uint32_t>(0)];
  auto const path_1_up =
      w.r_->unpack_shortcut<direction::kForward, true>(sc_1_up);
  ASSERT_EQ(path_1_up.size(), 1);
  ASSERT_EQ(path_1_up[static_cast<std::uint32_t>(0)], sc_1_up.exit_node_);

  auto const& sc_1_down =
      w.r_->cch_sc_down_[via_node_1][static_cast<std::uint32_t>(0)];
  auto const path_1_down =
      w.r_->unpack_shortcut<direction::kForward, false>(sc_1_down);
  ASSERT_EQ(path_1_down.size(), 1);
  ASSERT_EQ(path_1_down[static_cast<std::uint32_t>(0)], sc_1_down.exit_node_);

  auto const& sc_3_up =
      w.r_->cch_sc_up_[via_node_2][static_cast<std::uint32_t>(2)];
  auto const path_3_up =
      w.r_->unpack_shortcut<direction::kForward, true>(sc_3_up);
  ASSERT_EQ(path_3_up.size(), 2);
  ASSERT_EQ(
      path_3_up[static_cast<std::uint32_t>(0)],
      w.r_->cch_sc_down_[via_node_1][static_cast<std::uint32_t>(0)].exit_node_);
  ASSERT_EQ(
      path_3_up[static_cast<std::uint32_t>(1)],
      w.r_->cch_sc_up_[via_node_1][static_cast<std::uint32_t>(1)].exit_node_);

  auto const& sc_3_down = w.r_->cch_sc_down_[via_node_2][2];
  auto const path_3_down =
      w.r_->unpack_shortcut<direction::kForward, false>(sc_3_down);
  ASSERT_EQ(path_3_up.size(), 2);
  ASSERT_EQ(
      path_3_down[static_cast<std::uint32_t>(0)],
      w.r_->cch_sc_down_[via_node_1][static_cast<std::uint32_t>(1)].exit_node_);
  ASSERT_EQ(
      path_3_down[static_cast<std::uint32_t>(1)],
      w.r_->cch_sc_up_[via_node_1][static_cast<std::uint32_t>(0)].exit_node_);

  auto const& sc_8_up = w.r_->cch_sc_up_[via_node_3][2];
  auto const path_8_up =
      w.r_->unpack_shortcut<direction::kForward, true>(sc_8_up);
  ASSERT_EQ(path_8_up.size(), 3);
  ASSERT_EQ(
      path_8_up[static_cast<std::uint32_t>(0)],
      w.r_->cch_sc_down_[via_node_2][static_cast<std::uint32_t>(1)].exit_node_);
  ASSERT_EQ(
      path_8_up[static_cast<std::uint32_t>(1)],
      w.r_->cch_sc_down_[via_node_1][static_cast<std::uint32_t>(0)].exit_node_);
  ASSERT_EQ(
      path_8_up[static_cast<std::uint32_t>(2)],
      w.r_->cch_sc_up_[via_node_1][static_cast<std::uint32_t>(1)].exit_node_);

  auto const& sc_8_down = w.r_->cch_sc_down_[via_node_3][2];
  auto const path_8_down =
      w.r_->unpack_shortcut<direction::kForward, false>(sc_8_down);
  ASSERT_EQ(path_8_down.size(), 3);
  ASSERT_EQ(
      path_8_down[static_cast<std::uint32_t>(0)],
      w.r_->cch_sc_down_[via_node_1][static_cast<std::uint32_t>(1)].exit_node_);
  ASSERT_EQ(
      path_8_down[static_cast<std::uint32_t>(1)],
      w.r_->cch_sc_up_[via_node_1][static_cast<std::uint32_t>(0)].exit_node_);
  ASSERT_EQ(
      path_8_down[static_cast<std::uint32_t>(2)],
      w.r_->cch_sc_up_[via_node_2][static_cast<std::uint32_t>(1)].exit_node_);

  auto const& sc_9_up = w.r_->cch_sc_up_[via_node_5][5];
  auto const path_9_up =
      w.r_->unpack_shortcut<direction::kForward, true>(sc_9_up);
  ASSERT_EQ(path_9_up.size(), 5);
  ASSERT_EQ(
      path_9_up[static_cast<std::uint32_t>(0)],
      w.r_->cch_sc_down_[via_node_1][static_cast<std::uint32_t>(1)].exit_node_);
  ASSERT_EQ(
      path_9_up[static_cast<std::uint32_t>(1)],
      w.r_->cch_sc_up_[via_node_1][static_cast<std::uint32_t>(0)].exit_node_);
  ASSERT_EQ(
      path_9_up[static_cast<std::uint32_t>(2)],
      w.r_->cch_sc_up_[via_node_2][static_cast<std::uint32_t>(1)].exit_node_);
  ASSERT_EQ(
      path_9_up[static_cast<std::uint32_t>(3)],
      w.r_->cch_sc_down_[via_node_4][static_cast<std::uint32_t>(0)].exit_node_);
  ASSERT_EQ(
      path_9_up[static_cast<std::uint32_t>(4)],
      w.r_->cch_sc_up_[via_node_4][static_cast<std::uint32_t>(3)].exit_node_);

  auto const& sc_9_down = w.r_->cch_sc_down_[via_node_5][5];
  auto const path_9_down =
      w.r_->unpack_shortcut<direction::kForward, false>(sc_9_down);
  ASSERT_EQ(path_9_down.size(), 5);
  ASSERT_EQ(
      path_9_down[static_cast<std::uint32_t>(0)],
      w.r_->cch_sc_down_[via_node_4][static_cast<std::uint32_t>(3)].exit_node_);
  ASSERT_EQ(
      path_9_down[static_cast<std::uint32_t>(1)],
      w.r_->cch_sc_up_[via_node_4][static_cast<std::uint32_t>(0)].exit_node_);
  ASSERT_EQ(
      path_9_down[static_cast<std::uint32_t>(2)],
      w.r_->cch_sc_down_[via_node_2][static_cast<std::uint32_t>(1)].exit_node_);
  ASSERT_EQ(
      path_9_down[static_cast<std::uint32_t>(3)],
      w.r_->cch_sc_down_[via_node_1][static_cast<std::uint32_t>(0)].exit_node_);
  ASSERT_EQ(
      path_9_down[static_cast<std::uint32_t>(4)],
      w.r_->cch_sc_up_[via_node_1][static_cast<std::uint32_t>(1)].exit_node_);
}

TEST(extract, unpack_bigger_shortcut_backward) {
  auto const data_dir = "test/darmstadt-bismarckstr.osm.pbf";
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  if (!fs::exists(data_dir)) {
    GTEST_SKIP() << data_dir << " not found";
  }

  extract(false, data_dir, p, {}, true);
  auto w = ways{p, cista::mmap::protection::READ};

  std::uint32_t const via_node_1 = 83;
  std::uint32_t const via_node_2 = 134;

  if (w.r_->contraction_order_[via_node_1] != osr::node_idx_t{222} ||
      w.r_->contraction_order_[via_node_2] != osr::node_idx_t{226}) {
    GTEST_SKIP() << data_dir << " with invalid version";
  }

  auto const& sc_1_up =
      w.r_->cch_sc_up_[via_node_1][static_cast<std::uint32_t>(0)];
  auto const path_1_up =
      w.r_->unpack_shortcut<direction::kBackward, true>(sc_1_up);
  ASSERT_EQ(path_1_up.size(), 1);
  ASSERT_EQ(
      path_1_up[static_cast<std::uint32_t>(0)],
      w.r_->cch_sc_up_[via_node_1][static_cast<std::uint32_t>(0)].entry_node_);

  auto const& sc_1_down =
      w.r_->cch_sc_down_[via_node_1][static_cast<std::uint32_t>(0)];
  auto const path_1_down =
      w.r_->unpack_shortcut<direction::kBackward, false>(sc_1_down);
  ASSERT_EQ(path_1_down.size(), 1);
  ASSERT_EQ(path_1_down[static_cast<std::uint32_t>(0)], sc_1_down.entry_node_);

  auto const& sc_3_up =
      w.r_->cch_sc_up_[via_node_2][static_cast<std::uint32_t>(2)];
  auto const path_3_up =
      w.r_->unpack_shortcut<direction::kBackward, true>(sc_3_up);
  ASSERT_EQ(path_3_up.size(), 2);
  ASSERT_EQ(path_3_up[static_cast<std::uint32_t>(0)],
            w.r_->cch_sc_down_[via_node_1][static_cast<std::uint32_t>(0)]
                .entry_node_);
  ASSERT_EQ(
      path_3_up[static_cast<std::uint32_t>(1)],
      w.r_->cch_sc_up_[via_node_1][static_cast<std::uint32_t>(1)].entry_node_);

  auto const& sc_3_down =
      w.r_->cch_sc_down_[via_node_2][static_cast<std::uint32_t>(2)];
  auto const path_3_down =
      w.r_->unpack_shortcut<direction::kBackward, false>(sc_3_down);
  ASSERT_EQ(path_3_down.size(), 2);
  ASSERT_EQ(path_3_down[static_cast<std::uint32_t>(0)],
            w.r_->cch_sc_down_[via_node_1][static_cast<std::uint32_t>(1)]
                .entry_node_);
  ASSERT_EQ(
      path_3_down[static_cast<std::uint32_t>(1)],
      w.r_->cch_sc_up_[via_node_1][static_cast<std::uint32_t>(0)].entry_node_);
}