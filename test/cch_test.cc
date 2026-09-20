#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <fstream>
#include <random>
#include <thread>

#include "cista/mmap.h"

#include "utl/parallel_for.h"

#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/algorithms.h"
#include "osr/routing/cch.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

namespace {
constexpr auto const kUseMultithreading = true;
constexpr auto const kMaxMatchDistance = 100;
constexpr auto const kData = "test/a17-barrier-access-yes.osm.pbf";
constexpr auto const kDataDir = "test/a17-cch-final";

void load(std::string_view raw_data, std::string_view data_dir) {
  if (fs::exists(raw_data)) {
    auto const p = fs::path{data_dir};
    auto ec = std::error_code{};
    fs::remove_all(p, ec);
    fs::create_directories(p, ec);
    osr::extract(false, raw_data, data_dir, fs::path{});
  }
}

template <typename P>
void run(ways const& w,
         lookup const& l,
         search_profile const profile,
         unsigned const n_samples,
         unsigned const max_cost,
         direction const dir,
         typename P::parameters const& parameters = {},
         bool const multithreading = kUseMultithreading) {
  auto const from_tos = [&]() {
    auto prng = std::mt19937{};
    auto distr =
        std::uniform_int_distribution<std::uint32_t>{0, w.n_nodes() - 1};
    auto from_tos = std::vector<std::pair<node_idx_t, node_idx_t>>{};
    for (auto i = 0U; i != n_samples; ++i) {
      from_tos.emplace_back(distr(prng), distr(prng));
    }
    return from_tos;
  }();

  auto n_congruent = std::atomic<unsigned>{0U};
  auto n_empty_matches = std::atomic<unsigned>{0U};
  auto n_too_close = std::atomic<unsigned>{0U};
  auto n_routed = std::atomic<unsigned>{0U};
  auto reference_times = std::vector<std::chrono::steady_clock::duration>{};
  auto experiment_times = std::vector<std::chrono::steady_clock::duration>{};

  auto m = std::mutex{};
  auto warmed = std::once_flag{};

  auto const single_run = [&](std::pair<node_idx_t, node_idx_t> const from_to) {
    auto const from_node = from_to.first;
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_node = from_to.second;
    auto const to_loc = location{w.get_node_pos(to_node)};

    if (geo::distance(from_loc.pos_, to_loc.pos_) < 8.0) {
      ++n_too_close;
      return;
    }

    auto const node_pinned_matches =
        [&](location const& loc, node_idx_t const n, bool const reverse) {
          auto matches = l.match<P>(parameters, loc, reverse, dir,
                                    kMaxMatchDistance, nullptr);
          std::erase_if(matches, [&](auto const& wc) {
            return wc.left_.node_ != n && wc.right_.node_ != n;
          });
          if (matches.size() > 1) {
            matches.resize(1);
          }
          return matches;
        };
    auto const from_matches = node_pinned_matches(from_loc, from_node, false);
    auto const to_matches = node_pinned_matches(to_loc, to_node, true);
    if (from_matches.empty() || to_matches.empty()) {
      ++n_empty_matches;
    }

    auto const from_matches_span =
        std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

    auto const params = profile_parameters{parameters};

    if (!from_matches.empty() && !to_matches.empty()) {
      std::call_once(warmed, [&] {
        cch::route(params, w, l, profile, from_loc, to_loc, from_matches_span,
                   to_matches_span, max_cost, dir);
      });
    }
    auto const reference_start = std::chrono::steady_clock::now();
    auto const reference = [&]() -> std::optional<cost_t> {
      try {
        auto const p =
            route(params, w, l, profile, from_loc, to_loc, from_matches_span,
                  to_matches_span, max_cost, dir, nullptr, nullptr, nullptr,
                  routing_algorithm::kDijkstra);
        return p ? std::optional{p->cost_} : std::nullopt;
      } catch (std::exception const& ex) {
        fmt::println("dijkstra exception: {}", ex.what());
        throw ex;
      }
    }();
    auto const reference_time =
        std::chrono::steady_clock::now() - reference_start;

    get_dijkstra<P>().cost_.clear();
    auto result = std::optional<path>{};
    auto const experiment_start = std::chrono::steady_clock::now();
    auto const experiment = [&]() -> std::optional<cost_t> {
      try {
        auto const& p = result =
            route(params, w, l, profile, from_loc, to_loc, from_matches_span,
                  to_matches_span, max_cost, dir, nullptr, nullptr, nullptr,
                  routing_algorithm::kCCH);
        return p ? std::optional{p->cost_} : std::nullopt;
      } catch (std::exception const& ex) {
        fmt::println("cch exception: {}", ex.what());
        throw ex;
      }
    }();
    auto const experiment_time =
        std::chrono::steady_clock::now() - experiment_start;

    auto const& p = result;
    EXPECT_TRUE(get_dijkstra<P>().cost_.empty());
    if (p) {
      auto cost = std::uint64_t{};
      auto distance = 0.0;
      for (auto i = 0U; i != p->segments_.size(); ++i) {
        auto const& seg = p->segments_[i];
        cost += seg.cost_;
        distance += seg.dist_;
        ASSERT_FALSE(seg.polyline_.empty());
        if (i != 0U) {
          EXPECT_EQ(p->segments_[i - 1U].to_, seg.from_);
          EXPECT_LT(geo::distance(p->segments_[i - 1U].polyline_.back(),
                                  seg.polyline_.front()),
                    0.1);
        }
        if (seg.way_ != way_idx_t::invalid()) {
          EXPECT_LT(
              geo::distance(seg.polyline_.front(), w.get_node_pos(seg.from_)),
              0.1);
          EXPECT_LT(
              geo::distance(seg.polyline_.back(), w.get_node_pos(seg.to_)),
              0.1);
        }
      }
      EXPECT_EQ(cost, p->cost_);
      EXPECT_NEAR(distance, p->dist_, 2.0);
    }

    if (reference != experiment) {
      auto const print_result = [&](std::string_view name,
                                    std::optional<cost_t> const& c,
                                    auto const& t) {
        fmt::println(
            "{:10}: {:11} --> {:11} | {} | time: {}:{:0>3}:{:0>3} s", name,
            w.node_to_osm_[from_node], w.node_to_osm_[to_node],
            c ? fmt::format("cost: {:5}", *c) : "no result",
            std::chrono::duration_cast<std::chrono::seconds>(t).count(),
            std::chrono::duration_cast<std::chrono::milliseconds>(t).count() %
                1000,
            std::chrono::duration_cast<std::chrono::microseconds>(t).count() %
                1000);
      };

      print_result("dijkstra", reference, reference_time);
      print_result("cch", experiment, experiment_time);
    } else {
      ++n_congruent;
    }

    if (reference.has_value()) {
      ++n_routed;
    }

    if (!from_matches.empty() && !to_matches.empty()) {
      auto const guard = std::lock_guard{m};
      reference_times.emplace_back(reference_time);
      experiment_times.emplace_back(experiment_time);
    }
  };

  if (multithreading) {
    utl::parallel_for(from_tos, single_run);
  } else {
    std::for_each(begin(from_tos), end(from_tos), single_run);
  }

  auto const non_empty_congruent = n_congruent - n_empty_matches;
  auto const non_empty_samples = n_samples - n_empty_matches - n_too_close;

  EXPECT_EQ(non_empty_samples, non_empty_congruent);
  EXPECT_GT(non_empty_samples, 100U);
  EXPECT_GT(n_routed.load(), 20U);

  fmt::println("congruent on non-empty: {}/{} ({:3.1f}%), {} with a route",
               non_empty_congruent, non_empty_samples,
               (static_cast<double>(non_empty_congruent) /
                static_cast<double>(non_empty_samples)) *
                   100,
               n_routed.load());
  if (non_empty_congruent == non_empty_samples) {
    fmt::println(
        "speedup on non-empty: {:.2f}",
        static_cast<double>(
            std::reduce(begin(reference_times), end(reference_times)).count()) /
            static_cast<double>(
                std::reduce(begin(experiment_times), end(experiment_times))
                    .count()));
  }
}
}  // namespace

TEST(cch, flag_is_available) {
  EXPECT_EQ(routing_algorithm::kCCH, to_algorithm("cch"));
}

TEST(cch, a17_car) {
  auto const num_samples = 20000U;
  auto const max_cost = 3600U;
  auto constexpr dir = direction::kForward;

  ASSERT_TRUE(fs::exists(kData));
  load(kData, kDataDir);
  auto const w = osr::ways{kDataDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kDataDir, cista::mmap::protection::READ};

  ASSERT_TRUE(fs::exists(fs::path{kDataDir} / "cch.bin"));
  ASSERT_TRUE(cch::usable(w, search_profile::kCar, nullptr, nullptr, nullptr));
  EXPECT_FALSE(
      cch::usable(w, search_profile::kFoot, nullptr, nullptr, nullptr));

  run<car>(w, l, search_profile::kCar, num_samples, max_cost, dir);
}

TEST(cch, a17_bus) {
  auto const num_samples = 20000U;
  auto const max_cost = 3600U;
  auto constexpr dir = direction::kForward;

  ASSERT_TRUE(fs::exists(kData));
  load(kData, kDataDir);
  auto const w = osr::ways{kDataDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kDataDir, cista::mmap::protection::READ};

  ASSERT_TRUE(cch::usable(w, search_profile::kBus, nullptr, nullptr, nullptr));
  run<bus>(w, l, search_profile::kBus, num_samples, max_cost, dir);
}

TEST(cch, cost_limit_is_honoured) {
  ASSERT_TRUE(fs::exists(kData));
  load(kData, kDataDir);
  auto const w = osr::ways{kDataDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kDataDir, cista::mmap::protection::READ};

  auto const params = profile_parameters{car::parameters{}};
  auto prng = std::mt19937{};
  auto distr = std::uniform_int_distribution<std::uint32_t>{0, w.n_nodes() - 1};
  auto found = false;

  for (auto i = 0U; i != 2000U && !found; ++i) {
    auto const from = location{w.get_node_pos(node_idx_t{distr(prng)})};
    auto const to = location{w.get_node_pos(node_idx_t{distr(prng)})};
    auto const from_match =
        l.match<car>(car::parameters{}, from, false, direction::kForward,
                     kMaxMatchDistance, nullptr);
    auto const to_match =
        l.match<car>(car::parameters{}, to, true, direction::kForward,
                     kMaxMatchDistance, nullptr);
    if (from_match.empty() || to_match.empty()) {
      continue;
    }
    auto const full =
        cch::route(params, w, l, search_profile::kCar, from, to, from_match,
                   to_match, 3600U, direction::kForward);
    if (!full.has_value() || full->cost_ < 30U) {
      continue;
    }
    auto minimum = kInfeasible;
    for (auto const& a : from_match) {
      for (auto const& b : to_match) {
        auto const d = route(params, w, l, search_profile::kCar, from, to,
                             match_view_t{&a, 1U}, match_view_t{&b, 1U}, 3600U,
                             direction::kForward, nullptr, nullptr, nullptr,
                             routing_algorithm::kDijkstra);
        if (d) {
          minimum = std::min(minimum, d->cost_);
        }
      }
    }
    EXPECT_EQ(minimum, full->cost_);
    EXPECT_FALSE(cch::route(params, w, l, search_profile::kCar, from, to,
                            from_match, to_match,
                            static_cast<cost_t>(full->cost_ / 2U),
                            direction::kForward)
                     .has_value());
    EXPECT_FALSE(cch::route(params, w, l, search_profile::kCar, from, to,
                            from_match, to_match, full->cost_,
                            direction::kForward));
    ASSERT_TRUE(cch::route(params, w, l, search_profile::kCar, from, to,
                           from_match, to_match, full->cost_ + 1U,
                           direction::kForward));
    auto other = osr::ways{kDataDir, cista::mmap::protection::READ};
    for (auto& prop : other.r_->way_properties_) {
      prop.is_car_accessible_ = false;
    }
    auto const other_lookup =
        osr::lookup{other, kDataDir, cista::mmap::protection::READ};
    EXPECT_FALSE(cch::route(params, other, other_lookup, search_profile::kCar,
                            from, to, from_match, to_match, 3600U,
                            direction::kForward));
    auto const again =
        cch::route(params, w, l, search_profile::kCar, from, to, from_match,
                   to_match, 3600U, direction::kForward);
    ASSERT_TRUE(again);
    EXPECT_EQ(full->cost_, again->cost_);
    found = true;
  }
  EXPECT_TRUE(found);
}

TEST(cch, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto const num_samples = 10000U;
  auto const max_cost = 2 * 3600U;
  auto constexpr dir = direction::kForward;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run<car>(w, l, search_profile::kCar, num_samples, max_cost, dir);
  run<bus>(w, l, search_profile::kBus, num_samples, max_cost, dir);
  run<car>(w, l, search_profile::kCar, num_samples, max_cost,
           direction::kBackward);
  run<bus>(w, l, search_profile::kBus, num_samples, max_cost,
           direction::kBackward);
}

TEST(cch, hamburg) {
  auto const raw_data = "test/hamburg.osm.pbf";
  auto const data_dir = "test/hamburg";
  auto const num_samples = 5000U;
  auto const max_cost = 3 * 3600U;
  auto constexpr dir = direction::kForward;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run<car>(w, l, search_profile::kCar, num_samples, max_cost, dir);
  run<bus>(w, l, search_profile::kBus, num_samples, max_cost, dir);
}

TEST(cch, switzerland) {
  auto const raw_data = "test/switzerland.osm.pbf";
  auto const data_dir = "test/switzerland";
  auto const num_samples = 1000U;
  auto const max_cost = 5 * 3600U;
  auto constexpr dir = direction::kForward;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run<car>(w, l, search_profile::kCar, num_samples, max_cost, dir, {}, false);
  run<bus>(w, l, search_profile::kBus, num_samples, max_cost, dir, {}, false);
}

TEST(cch, a17_backward) {
  load(kData, kDataDir);
  auto const w = osr::ways{kDataDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kDataDir, cista::mmap::protection::READ};
  run<car>(w, l, search_profile::kCar, 20000U, 3600U, direction::kBackward);
  run<bus>(w, l, search_profile::kBus, 20000U, 3600U, direction::kBackward);
}

TEST(cch, restrictions_and_customization) {
  auto const data_dir = "test/cch-restrictions";
  load("test/map.osm", data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};
  ASSERT_TRUE(utl::any_of(w.r_->node_restrictions_,
                          [](auto const rs) { return !rs.empty(); }));
  for (auto const dir : {direction::kForward, direction::kBackward}) {
    run<car>(w, l, search_profile::kCar, 5000U, 3600U, dir);
    run<bus>(w, l, search_profile::kBus, 5000U, 3600U, dir);
    run<bus>(w, l, search_profile::kBus, 5000U, 3600U, dir,
             bus::parameters{.uturn_penalty_ = 0U,
                             .private_gate_penalty_ = 200U,
                             .slow_turn_penalty_ = 80U,
                             .sharp_turn_penalty_ = 150U});
    run<bus>(w, l, search_profile::kBus, 5000U, 3600U, dir);
  }
}

TEST(cch, metric_cache_and_scratch_reuse) {
  auto const small_dir = "test/cch-restrictions";
  load("test/map.osm", small_dir);
  load(kData, kDataDir);
  auto const small = ways{small_dir, cista::mmap::protection::READ};
  auto const large = ways{kDataDir, cista::mmap::protection::READ};
  auto const small_lookup =
      lookup{small, small_dir, cista::mmap::protection::READ};
  auto const large_lookup =
      lookup{large, kDataDir, cista::mmap::protection::READ};
  ASSERT_NE(small.n_nodes(), large.n_nodes());
  auto const custom = bus::parameters{.uturn_penalty_ = 0U,
                                      .private_gate_penalty_ = 200U,
                                      .slow_turn_penalty_ = 80U,
                                      .sharp_turn_penalty_ = 150U};

  // Stay on one thread so switching maps and metrics reuses the same scratch.
  for (auto round = 0U; round != 3U; ++round) {
    for (auto const use_small : {false, true}) {
      auto const& w = use_small ? small : large;
      auto const& l = use_small ? small_lookup : large_lookup;
      for (auto const dir : {direction::kForward, direction::kBackward}) {
        run<car>(w, l, search_profile::kCar, 5000U, 3600U, dir, {}, false);
        run<bus>(w, l, search_profile::kBus, 5000U, 3600U, dir, {}, false);
        run<bus>(w, l, search_profile::kBus, 5000U, 3600U, dir, custom, false);
      }
    }
  }
  for (auto const penalty : {40U, 0U, 120U}) {
    run<bus>(large, large_lookup, search_profile::kBus, 5000U, 3600U,
             direction::kForward, bus::parameters{.uturn_penalty_ = penalty},
             false);
  }
}

TEST(cch, concurrent_metric_eviction) {
  load("test/map.osm", "test/cch-restrictions");
  auto const w = ways{"test/cch-restrictions", cista::mmap::protection::READ};
  auto const l =
      lookup{w, "test/cch-restrictions", cista::mmap::protection::READ};
  auto workers = std::vector<std::jthread>{};
  for (auto worker = 0U; worker != 4U; ++worker) {
    workers.emplace_back([&, worker] {
      for (auto round = 0U; round != 3U; ++round) {
        auto const dir =
            round % 2U == 0U ? direction::kForward : direction::kBackward;
        run<car>(w, l, search_profile::kCar, 1000U, 3600U, dir, {}, false);
        run<bus>(w, l, search_profile::kBus, 1000U, 3600U, dir,
                 bus::parameters{.uturn_penalty_ = worker * 40U}, false);
      }
    });
  }
}

TEST(cch, invalid_topology_and_fallback) {
  auto const data_dir = "test/cch-invalid";
  load(kData, data_dir);
  auto const file = fs::path{data_dir} / cch::kFileName;
  {
    auto f =
        std::fstream{file, std::ios::binary | std::ios::in | std::ios::out};
    f.seekp(3 * sizeof(std::uint64_t));
    auto const invalid_rank = std::numeric_limits<std::uint32_t>::max();
    f.write(reinterpret_cast<char const*>(&invalid_rank), sizeof(invalid_rank));
  }
  auto const check = [&] {
    auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
    auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};
    EXPECT_FALSE(
        cch::usable(w, search_profile::kCar, nullptr, nullptr, nullptr));
    auto const from = location{w.get_node_pos(node_idx_t{0U})};
    auto const to = location{w.get_node_pos(node_idx_t{w.n_nodes() - 1U})};
    auto const d = route(car::parameters{}, w, l, search_profile::kCar, from,
                         to, 3600U, direction::kForward, 100);
    auto const c = route(car::parameters{}, w, l, search_profile::kCar, from,
                         to, 3600U, direction::kForward, 100, nullptr, nullptr,
                         nullptr, routing_algorithm::kCCH);
    ASSERT_EQ(d.has_value(), c.has_value());
    if (d) {
      EXPECT_EQ(d->cost_, c->cost_);
    }
  };
  check();
  fs::resize_file(file, 25U);
  check();
  fs::remove(file);
  check();
}
