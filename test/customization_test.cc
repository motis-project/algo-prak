#include "gtest/gtest.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string_view>

#include "cista/mmap.h"

#include "fmt/core.h"

#include "osr/extract/extract.h"

#include "osr/lookup.h"
#include "osr/types.h"
#include "osr/ways.h"

#include "osr/routing/profiles/car.h"

#include "osr/cch_preprocessing/preprocess.h"
#include "osr/cch_preprocessing/preprocessed_data.h"

namespace fs = std::filesystem;
using namespace osr;

namespace customization_test {

void load_data(std::string_view raw_data, std::string_view data_dir) {
  if (fs::exists(raw_data)) {
    auto const p = fs::path{data_dir};
    auto ec = std::error_code{};
    fs::remove_all(p, ec);
    fs::create_directories(p, ec);
    osr::extract(false, raw_data, data_dir, fs::path{});
  }
}

void load_customized_cost(std::string_view raw_data,
                          std::string_view data_dir,
                          ways const& w) {
  if (fs::exists(raw_data)) {
    fmt::print("Preprocessing CCH for {}...\n", data_dir);
    osr::cch_preprocessing::preprocess(fs::path{data_dir}, w);
    fmt::print("Done extracting and preprocessing {} to {}.\n", raw_data,
               data_dir);
    osr::cch_preprocessing::preprocessed_data::load_metric_independent(
        fs::path{data_dir});
    osr::cch_preprocessing::preprocessed_data::load_customized_cost<car>(
        fs::path{data_dir});
  }
}

void valid_test(std::string_view data_dir, ways const& w) {
  auto const& cc =
      osr::cch_preprocessing::preprocessed_data::get_customized_cost<car>();
  auto const& ordering =
      osr::cch_preprocessing::preprocessed_data::get_ordering();

  auto const checkExists = [&](auto const from, auto const to,
                               auto const cost) -> bool {
    bool found = false;
    if (from < to) {
      cc.for_each_edge<true>(
          cc.get_flatten_node_idx(from), [&](auto const&, auto const& e) {
            if (e.from_ == from && e.to_ == to && e.cost_ == cost) {
              found = true;
            }
          });
    } else {
      cc.for_each_edge<false>(
          cc.get_flatten_node_idx(from), [&](auto const&, auto const& e) {
            if (e.from_ == from && e.to_ == to && e.cost_ == cost) {
              found = true;
            }
          });
    }
    return found;
  };

  fmt::print("Validating CCH for {}...\n", data_dir);

  ASSERT_EQ(ordering.size(), w.n_nodes());
  ASSERT_EQ(cc.virtual_nodes_.size(), w.n_nodes());
  ASSERT_EQ(cc.idx_ranges_.size(), w.n_nodes());

  fmt::print("Found {} virtual nodes and {} extended edges.\n",
             cc.virtual_nodes_.size(), cc.extended_edges_.size());

  for (auto const& way : w.r_->way_nodes_) {
    for (auto const& node : way) {
      car::resolve_all(*w.r_, node, kNoLevel, [&](auto const& n) {
        auto const idx = cc.get_virtual_node_idx(n, ordering);
        ASSERT_TRUE(idx != osr::cch_preprocessing::ext_node::invalid());
      });
    }
  }

  fmt::print("All virtual nodes have valid indices.\n");

  std::uint32_t total_idxes = 0;
  node_idx_t curr_idx = static_cast<node_idx_t>(0);
  for (auto const& idx_pointer : cc.idx_ranges_) {
    ASSERT_TRUE(idx_pointer >= total_idxes);
    ASSERT_EQ(total_idxes, idx_pointer);
    total_idxes += cc.virtual_nodes_[curr_idx.v_].size();
    ++curr_idx;
  }

  auto const max_idx = static_cast<node_idx_t>(cc.idx_ranges_.back() +
                                               cc.virtual_nodes_.back().size());
  fmt::print("Found maximal flattened idx {}.\n", max_idx);
  ASSERT_EQ(max_idx, cc.upward_edges_.size());
  ASSERT_EQ(max_idx, cc.downward_edges_.size());

  std::uint32_t total_upward_edges = 0;
  std::uint32_t total_downward_edges = 0;
  for (auto const& edges : cc.upward_edges_) {
    total_upward_edges += edges.second - edges.first;
  }
  for (auto const& edges : cc.downward_edges_) {
    total_downward_edges += edges.second - edges.first;
  }
  fmt::print("Found total {} upward edges and {} downward edges.\n",
             total_upward_edges, total_downward_edges);
  ASSERT_EQ(total_upward_edges + total_downward_edges,
            cc.extended_edges_.size());

  std::uint32_t total_passed = 0;
  for (auto const& edge : cc.extended_edges_) {
    total_passed += checkExists(edge.from_, edge.to_, edge.cost_) ? 1 : 0;
  }
  ASSERT_EQ(total_passed, cc.extended_edges_.size());

  for (auto const& edge : cc.extended_edges_) {
    if (!edge.is_original() || edge.cost_ == kInfeasible) {
      continue;
    }
    auto const from = cc.get_virtual_node(edge.from_);
    auto const to = cc.get_virtual_node(edge.to_);

    bool found = false;
    car::template adjacent<direction::kForward, false>(
        car::parameters{}, *w.r_, from, nullptr, nullptr, nullptr,
        [&](car::node const curr_to, std::uint32_t const curr_cost, distance_t,
            way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation,
            bool const) { found |= curr_to == to && curr_cost == edge.cost_; });
    ASSERT_TRUE(found);
  }

  for (auto const& edge : cc.extended_edges_) {
    if (edge.is_original()) {
      continue;
    }

    ASSERT_TRUE(0 <= edge.cost_ && edge.cost_ < kInfeasible);
    auto const trace = edge.traceback_;
    ASSERT_FALSE(edge.is_original());
    auto const from = cc.get_edge(trace.first);
    auto const to = cc.get_edge(trace.second);
    auto const total_cost = from.cost_ + to.cost_;

    ASSERT_EQ(edge.from_, from.from_);
    ASSERT_EQ(edge.to_, to.to_);
    ASSERT_EQ(from.to_, to.from_);
    ASSERT_EQ(edge.cost_, total_cost);
  }

  fmt::print("All extended edges are valid and have correct costs.\n");

  fmt::print("tree size: {}\n", cc.specialized_elimination_tree_.size());

  // for (std::uint32_t i = 0U; i < std::min(ordering.size(), size_t(20)); ++i)
  // {
  //   fmt::print("{}'s parent is {}.\n", i,
  //              cc.specialized_elimination_tree_.at(i));
  // }

  for (auto const& [idx, sub_idxes] : utl::enumerate(cc.virtual_nodes_)) {
    for (auto const& [sub_idx, node] : utl::enumerate(sub_idxes)) {
      std::uint32_t const ext_node = static_cast<std::uint32_t>(
          cc.idx_ranges_.at(static_cast<std::uint32_t>(idx)) + sub_idx);
      auto const& par = cc.get_parent(ext_node);

      if (sub_idx < sub_idxes.size() - 1U) {
        std::uint32_t const nxt = static_cast<std::uint32_t>(
            cc.idx_ranges_.at(static_cast<std::uint32_t>(idx)) + sub_idx + 1U);
        ASSERT_EQ(par.v_, nxt);
      } else if (cc.upward_edges_.at(ext_node).first !=
                 cc.upward_edges_.at(ext_node).second) {
        auto min_node = std::numeric_limits<std::uint32_t>::max();
        cc.for_each_edge<true>(ext_node, [&](auto const&, auto const& e) {
          auto const to = cc.get_flatten_node_idx(e.to_);
          if (to < min_node) {
            min_node = to;
          }
        });
        utl::verify(par != node_idx_t::invalid(),
                    "Parent node for extended node {} is missing", ext_node);
        utl::verify(min_node >= par.v_,
                    "Parent node {} is less than minimal child node {} for "
                    "extended node {}",
                    par, min_node, ext_node);
      }
    }
  }
}
}  // namespace customization_test

TEST(customization, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  customization_test::load_data(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};
  customization_test::load_customized_cost(raw_data, data_dir, w);

  customization_test::valid_test(data_dir, w);
}

TEST(customization, hamburg) {
  auto const raw_data = "test/hamburg.osm.pbf";
  auto const data_dir = "test/hamburg";

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  customization_test::load_data(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};
  customization_test::load_customized_cost(raw_data, data_dir, w);

  customization_test::valid_test(data_dir, w);
}