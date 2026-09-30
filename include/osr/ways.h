#pragma once

#if defined(_WIN32) || defined(_WIN64)
#include "windows.h"

#include "Memoryapi.h"
#define mlock(addr, size) VirtualLock((LPVOID)addr, (SIZE_T)size)
#else
#include <sys/mman.h>
#endif
#include <filesystem>
#include <ranges>

#include "fmt/ranges.h"
#include "fmt/std.h"

#include "osmium/osm/way.hpp"

#include "cista/memory_holder.h"
#include "cista/reflection/comparable.h"

#include "utl/enumerate.h"
#include "utl/equal_ranges_linear.h"
#include "utl/helpers/algorithm.h"
#include "utl/progress_tracker.h"
#include "utl/timer.h"
#include "utl/verify.h"
#include "utl/zip.h"

#include "osr/point.h"
#include "osr/routing/turns.h"
#include "osr/shortcut.h"
#include "osr/types.h"
#include "osr/util/multi_counter.h"

namespace osr {

struct resolved_restriction {
  enum class type { kNo, kOnly } type_;
  way_idx_t from_, to_;
  node_idx_t via_;
  bool applies_to_bus_{true};
};

struct restriction {
  friend bool operator==(restriction, restriction) = default;

  template <std::size_t NMaxTypes>
  friend constexpr auto static_type_hash(
      restriction const*, cista::hash_data<NMaxTypes> h) noexcept {
    return h.combine(cista::hash("restriction v1.1"));
  }

  template <typename Ctx>
  friend void serialize(Ctx&, restriction const*, cista::offset_t) {}

  template <typename Ctx>
  friend void deserialize(Ctx const&, restriction*) {}

  way_pos_t from_ : 4;
  way_pos_t to_ : 4;
  way_pos_t applies_to_bus_ : 1;
};

struct way_properties {
  constexpr bool is_accessible() const {
    return is_car_accessible() || is_bike_accessible() ||
           is_foot_accessible() || is_bus_accessible() ||
           is_bus_accessible_with_penalty() || is_railway_accessible() ||
           is_railway_accessible_with_penalty() || is_ferry_accessible();
  }
  constexpr bool is_car_accessible() const { return is_car_accessible_; }
  constexpr bool is_bike_accessible() const { return is_bike_accessible_; }
  constexpr bool is_foot_accessible() const { return is_foot_accessible_; }
  constexpr bool is_bus_accessible() const { return is_bus_accessible_; }
  constexpr bool is_bus_accessible_with_penalty() const {
    return is_bus_accessible_with_penalty_;
  }
  constexpr bool is_railway_accessible() const {
    return is_railway_accessible_;
  }
  constexpr bool is_railway_accessible_with_penalty() const {
    return is_railway_accessible_with_penalty_;
  }
  constexpr bool is_ferry_accessible() const { return is_ferry_accessible_; }
  constexpr bool is_big_street() const { return is_big_street_; }
  constexpr bool is_destination() const { return is_destination_; }
  constexpr bool is_oneway_car() const { return is_oneway_car_; }
  constexpr bool is_oneway_bike() const { return is_oneway_bike_; }
  constexpr bool is_oneway_bus_psv() const { return is_oneway_bus_psv_; }
  constexpr bool is_elevator() const { return is_elevator_; }
  constexpr bool is_steps() const { return is_steps_; }
  constexpr bool is_ramp() const { return is_ramp_; }
  constexpr bool is_parking() const { return is_parking_; }
  constexpr bool has_toll() const { return has_toll_; }
  constexpr bool is_sidewalk_separate() const { return is_sidewalk_separate_; }
  constexpr bool in_route() const { return in_route_; }
  constexpr std::uint16_t max_speed_km_per_h() const {
    return to_kmh(static_cast<speed_limit>(speed_limit_));
  }
  constexpr float max_speed_s_per_m() const {
    return to_seconds_per_meter(static_cast<speed_limit>(speed_limit_));
  }
  constexpr level_t from_level() const { return level_t{from_level_}; }
  constexpr level_t to_level() const { return level_t{to_level_}; }

  template <std::size_t NMaxTypes>
  friend constexpr auto static_type_hash(
      way_properties const*, cista::hash_data<NMaxTypes> h) noexcept {
    return h.combine(cista::hash("way_properties v1.1"));
  }

  template <typename Ctx>
  friend void serialize(Ctx&, way_properties const*, cista::offset_t) {}

  template <typename Ctx>
  friend void deserialize(Ctx const&, way_properties*) {}

  std::uint8_t is_foot_accessible_ : 1;
  std::uint8_t is_bike_accessible_ : 1;
  std::uint8_t is_car_accessible_ : 1;
  std::uint8_t is_destination_ : 1;
  std::uint8_t is_oneway_car_ : 1;
  std::uint8_t is_oneway_bike_ : 1;
  std::uint8_t is_elevator_ : 1;
  std::uint8_t is_steps_ : 1;

  std::uint8_t speed_limit_ : 3;
  std::uint8_t is_platform_ : 1;  // only used during extract
  std::uint8_t is_parking_ : 1;
  std::uint8_t is_ramp_ : 1;
  std::uint8_t is_sidewalk_separate_ : 1;
  std::uint8_t motor_vehicle_no_ : 1;

  std::uint8_t from_level_ : 6;
  std::uint8_t has_toll_ : 1;
  std::uint8_t is_big_street_ : 1;

  std::uint8_t to_level_ : 6;
  std::uint8_t is_bus_accessible_ : 1;
  std::uint8_t in_route_ : 1;

  std::uint8_t is_railway_accessible_ : 1;
  std::uint8_t is_oneway_bus_psv_ : 1;
  std::uint8_t is_incline_down_ : 1;
  std::uint8_t is_bus_accessible_with_penalty_ : 1;
  std::uint8_t is_ferry_accessible_ : 1;
  std::uint8_t is_railway_accessible_with_penalty_ : 1;
};

static_assert(sizeof(way_properties) == 5);

struct node_properties {
  constexpr bool is_car_accessible() const { return is_car_accessible_; }
  constexpr bool is_bike_accessible() const { return is_bike_accessible_; }
  constexpr bool is_walk_accessible() const { return is_foot_accessible_; }
  constexpr bool is_bus_accessible() const { return is_bus_accessible_; }
  constexpr bool is_bus_accessible_with_penalty() const {
    return is_bus_accessible_with_penalty_;
  }
  constexpr bool is_elevator() const { return is_elevator_; }
  constexpr bool is_multi_level() const { return is_multi_level_; }
  constexpr bool is_entrance() const { return is_entrance_; }
  constexpr bool is_parking() const { return is_parking_; }

  constexpr level_t from_level() const { return level_t{from_level_}; }
  constexpr level_t to_level() const { return level_t{to_level_}; }

  template <std::size_t NMaxTypes>
  friend constexpr auto static_type_hash(
      node_properties const*, cista::hash_data<NMaxTypes> h) noexcept {
    return h.combine(cista::hash("node_properties v1"));
  }

  template <typename Ctx>
  friend void serialize(Ctx&, node_properties const*, cista::offset_t) {}

  template <typename Ctx>
  friend void deserialize(Ctx const&, node_properties*) {}

  std::uint8_t from_level_ : 6;
  std::uint8_t is_foot_accessible_ : 1;
  std::uint8_t is_bike_accessible_ : 1;

  std::uint8_t is_car_accessible_ : 1;
  std::uint8_t is_bus_accessible_ : 1;
  std::uint8_t is_elevator_ : 1;
  std::uint8_t is_entrance_ : 1;
  std::uint8_t is_multi_level_ : 1;
  std::uint8_t is_parking_ : 1;

  std::uint8_t to_level_ : 6;
  std::uint8_t is_bus_accessible_with_penalty_ : 1;
};

static_assert(sizeof(node_properties) == 3);

struct ways {
  ways(std::filesystem::path, cista::mmap::protection);

  void add_restriction(std::vector<resolved_restriction>&);
  void compute_big_street_neighbors();
  void connect_ways();
  void compute_turn_bearings();
  void build_components_and_importance(bool const with_cch);

  std::optional<way_idx_t> find_way(osm_way_idx_t const i) {
    auto const it = std::lower_bound(begin(way_osm_idx_), end(way_osm_idx_), i);
    return it != end(way_osm_idx_) && *it == i
               ? std::optional{way_idx_t{
                     std::distance(begin(way_osm_idx_), it)}}
               : std::nullopt;
  }

  bool is_additional_node(osr::node_idx_t const n) const {
    return n != node_idx_t::invalid() && n >= n_nodes();
  }

  std::optional<node_idx_t> find_node_idx(osm_node_idx_t const i) const {
    auto const it = std::lower_bound(begin(node_to_osm_), end(node_to_osm_), i,
                                     [](auto&& a, auto&& b) { return a < b; });
    if (it == end(node_to_osm_) || *it != i) {
      return std::nullopt;
    }
    return {node_idx_t{static_cast<node_idx_t::value_t>(
        std::distance(begin(node_to_osm_), it))}};
  }

  node_idx_t get_node_idx(osm_node_idx_t const i) const {
    auto const j = find_node_idx(i);
    utl::verify(j.has_value(), "osm node {} not found", i);
    return *j;
  }

  point get_node_pos(node_idx_t const i) const {
    return r_->node_positions_.at(i);
  }

  std::size_t get_polyline_node_idx(
      way_idx_t const way, std::uint16_t const target_routing_idx) const;

  cista::mmap mm(char const* file) {
    return cista::mmap{(p_ / file).generic_string().c_str(), mode_};
  }

  void sync();

  way_idx_t::value_t n_ways() const { return way_osm_idx_.size(); }
  node_idx_t::value_t n_nodes() const { return node_to_osm_.size(); }

  std::optional<std::string_view> get_access_restriction(way_idx_t) const;

  std::filesystem::path p_;
  cista::mmap::protection mode_;

  struct routing {
    static constexpr auto const kMode =
        cista::mode::WITH_INTEGRITY | cista::mode::WITH_STATIC_VERSION;

    way_pos_t get_way_pos(node_idx_t const node, way_idx_t const way) const {
      auto const ways = node_ways_[node];
      for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
        if (ways[i] == way) {
          return i;
        }
      }
      return 0U;
    }

    way_pos_t get_way_pos(node_idx_t const node,
                          way_idx_t const way,
                          std::uint16_t const node_in_way_idx) const {
      auto const ways = node_ways_[node];
      for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
        if (ways[i] == way &&
            (i + 1U == ways.size() || ways[i] != ways[i + 1U] ||
             node_in_way_idx_[node][i] == node_in_way_idx)) {
          return i;
        }
      }
      return 0U;
    }

    template <direction SearchDir, bool IsBus = false>
    bool is_restricted(node_idx_t const n,
                       std::uint8_t const from,
                       std::uint8_t const to) const {
      if (!node_is_restricted_[n]) {
        return false;
      }

      auto const r = node_restrictions_[n];
      auto const from_way =
          SearchDir == direction::kForward ? way_pos_t{from} : way_pos_t{to};
      auto const to_way =
          SearchDir == direction::kForward ? way_pos_t{to} : way_pos_t{from};

      return utl::any_of(r, [&](restriction const& x) {
        if constexpr (IsBus) {
          return x.from_ == from_way && x.to_ == to_way && x.applies_to_bus_;
        } else {
          return x.from_ == from_way && x.to_ == to_way;
        }
      });
    }

    template <bool IsBus = false>
    bool is_restricted(node_idx_t const n,
                       std::uint8_t const from,
                       std::uint8_t const to,
                       direction const search_dir) const {
      return search_dir == direction::kForward
                 ? is_restricted<direction::kForward, IsBus>(n, from, to)
                 : is_restricted<direction::kBackward, IsBus>(n, from, to);
    }

    bool is_loop(way_idx_t const w) const {
      return way_nodes_[w].back() == way_nodes_[w].front();
    }

    distance_t get_way_node_distance(way_idx_t const way,
                                     std::uint16_t const node) const {
      auto const v = way_node_dist_[way][node];
      if (v != std::numeric_limits<std::uint16_t>::max()) {
        [[likely]] return distance_t{v};
      } else {
        auto const it = std::lower_bound(begin(long_way_node_dist_),
                                         end(long_way_node_dist_),
                                         long_distance{way, node, 0U});
        if (it != end(long_way_node_dist_) && it->way_ == way &&
            it->node_ == node) {
          return it->distance_;
        }
        throw utl::fail("long distance not found for way {} node {}",
                        to_idx(way), node);
      }
    }

    quantized_angle_t get_turn_angle(node_idx_t const n,
                                     way_pos_t const from,
                                     direction const from_dir,
                                     way_pos_t const to,
                                     direction const to_dir) const {
      return osr::get_turn_angle(node_turn_bearings_[n][from], from_dir,
                                 node_turn_bearings_[n][to], to_dir);
    }

    // return the shortcut between the virtual node from and the node to
    template <bool IsUp>
    cch::packed_shortcut get_packed_shortcut(node_idx_t const& from,
                                             way_pos_t const& to_way,
                                             direction const& to_dir,
                                             node_idx_t const& to) const {
      auto const& from_rank = node_importance_[from];
      if (from != to) {
        auto const t_idx = get_target_idx(from, to);
        return IsUp ? cch_sc_up_[from_rank][t_idx]
                    : cch_sc_down_[from_rank][t_idx];
      } else {
        for (auto const sc : cch_sc_self_[from_rank]) {
          auto const& e = IsUp ? sc.exit_node_ : sc.entry_node_;
          if (e.n_ == to && e.way_ == to_way && e.dir_ == to_dir) {
            return sc;
          }
        }
        return cch::packed_shortcut::invalid();
        throw utl::fail("[GET PACKED SHORTCUT] Got no matching self shorcut");
      }
    }

    // return the index of a target (upper neighbor) in from
    std::uint32_t get_target_idx(node_idx_t const& from,
                                 node_idx_t const& to) const {
      auto const& from_rank = node_importance_[from];
      for (auto const [idx, target] : utl::enumerate(sc_targets_[from_rank])) {
        if (to == target) {
          return static_cast<std::uint32_t>(idx);
          return static_cast<std::uint32_t>(idx);
        }
      }
      throw utl::fail("Node {} has not target {}", from, to);
    }

    // after unpacking the shortcut path return the edge cost for
    // specific edge in the path.
    template <bool IsUp>
    cost_t get_edge_cost(node_idx_t const& from,
                         way_pos_t const& from_way,
                         direction const& from_dir,
                         node_idx_t const& to,
                         cost_t const& penalty) const {
      auto const& from_rank = node_importance_[from];
      auto const& to_rank = node_importance_[to];

      if (from_rank < to_rank) {
        auto const& shortcut =
            IsUp ? get_packed_shortcut<true>(from, from_way, from_dir, to)
                 : get_packed_shortcut<false>(from, from_way, from_dir, to);
        utl::verify(cch_true_edge(shortcut),
                    "[EDGE COST 1] Try to get cost of unreal edge");
        auto const t_idx = get_target_idx(from, to);
        auto cost = IsUp ? cch_cost_up_[from_rank][t_idx]
                         : cch_cost_down_[from_rank][t_idx];
        utl::verify(cost != osr::kInfeasible,
                    "[EDGE COST 2] Try to get unreal cost");
        auto const& tp = IsUp ? shortcut.entry_node_ : shortcut.exit_node_;
        utl::verify(tp.n_ == from, "[EDGE COST 3] Invalid touching point");
        if (from_way == tp.way_ && from_dir == opposite(tp.dir_)) {
          cost += penalty;
        }
        return cost;
      }

      if (to_rank < from_rank) {
        auto const& shortcut =
            IsUp ? get_packed_shortcut<false>(to, from_way, from_dir, from)
                 : get_packed_shortcut<true>(to, from_way, from_dir, from);
        utl::verify(cch_true_edge(shortcut),
                    "[EDGE COST 4] Try to get cost of unreal edge");
        auto const t_idx = get_target_idx(to, from);
        auto cost = IsUp ? cch_cost_down_[to_rank][t_idx]
                         : cch_cost_up_[to_rank][t_idx];
        utl::verify(cost != osr::kInfeasible,
                    "[EDGE COST 5] Try to get unreal cost");
        auto const& tp = IsUp ? shortcut.entry_node_ : shortcut.exit_node_;
        utl::verify(tp.n_ == from, "[EDGE COST 6] Invalid touching point");
        if (from_way == tp.way_ && from_dir == opposite(tp.dir_)) {
          cost += penalty;
        }
        return cost;
      }
      throw utl::fail("Tried to get edge cost node to node");
    }

    // check if the shortcut consists of one (true) or multiple edges (false)
    bool cch_true_edge(cch::packed_shortcut const& sc) const {
      utl::verify(
          sc.is_valid(),
          "[CCH EDGE CHECK] Got invalid Shortcut in path. Failed unpacking");

      if (sc.down_ == 0U && sc.up_ == 0U && sc.via_rank_ == 0U &&
          sc.u_turn_penalty_ == cost_t{0U}) {
        return true;
      }

      return false;
    }

    // unpack a shortcut recursively here:
    template <direction PathDir, bool IsUp>
    osr::vec<cch::target_node> unpack_shortcut(
        cch::packed_shortcut const& sc) const {
      auto const is_fwd = PathDir == direction::kForward;

      // checke ob der shortcut eine echte Kante ist:
      if (cch_true_edge(sc)) {
        osr::vec<cch::target_node> unpacked_sc = {};
        auto const& entry_rank = IsUp ? node_importance_[sc.entry_node_.n_]
                                      : node_importance_[sc.exit_node_.n_];
        auto const& target_idx =
            IsUp ? get_target_idx(sc.entry_node_.n_, sc.exit_node_.n_)
                 : get_target_idx(sc.exit_node_.n_, sc.entry_node_.n_);
        auto const& edge_cost = IsUp ? cch_cost_up_[entry_rank][target_idx]
                                     : cch_cost_down_[entry_rank][target_idx];
        utl::verify(edge_cost != osr::kInfeasible,
                    "[unpack shortcut] found invalid costs during unpacking");

        if (is_fwd) {
          unpacked_sc.push_back(sc.exit_node_);
        } else {
          unpacked_sc.push_back(sc.entry_node_);
        }

        return unpacked_sc;
      }

      // Falls keine direkte Kante, entpacke rekursiv weiter
      auto const& to_target = IsUp ? cch_sc_up_[sc.via_rank_][sc.up_]
                                   : cch_sc_down_[sc.via_rank_][sc.down_];
      auto const& to_via = IsUp ? cch_sc_down_[sc.via_rank_][sc.down_]
                                : cch_sc_up_[sc.via_rank_][sc.up_];

      auto unpacked_to_target = this->unpack_shortcut<PathDir, IsUp>(to_target);
      auto unpacked_to_via = this->unpack_shortcut<PathDir, !IsUp>(to_via);

      if constexpr (IsUp) {
        unpacked_to_via.insert(unpacked_to_via.end(),
                               unpacked_to_target.begin(),
                               unpacked_to_target.end());
      } else {
        unpacked_to_target.insert(unpacked_to_target.end(),
                                  unpacked_to_via.begin(),
                                  unpacked_to_via.end());
      }

      return IsUp ? unpacked_to_via : unpacked_to_target;
    }

    static cista::wrapped<routing> read(std::filesystem::path const&);
    void write(std::filesystem::path const&) const;

    struct long_distance {
      CISTA_COMPARABLE()

      way_idx_t way_{};
      std::uint16_t node_{};
      distance_t distance_{};
    };

    vec_map<node_idx_t, node_properties> node_properties_;
    vec_map<way_idx_t, way_properties> way_properties_;

    vecvec<way_idx_t, node_idx_t> way_nodes_;
    vecvec<way_idx_t, std::uint16_t> way_node_dist_;
    vec<long_distance> long_way_node_dist_;

    vecvec<node_idx_t, way_idx_t> node_ways_;
    vecvec<node_idx_t, std::uint16_t> node_in_way_idx_;
    vecvec<node_idx_t, turn_bearing> node_turn_bearings_;

    bitvec<node_idx_t> node_is_restricted_;
    vecvec<node_idx_t, restriction> node_restrictions_;

    vec_map<node_idx_t, point> node_positions_;
    vec_map<node_idx_t, std::uint32_t> node_importance_;

    // CCH Relevant information stored in ways:
    vec<vec<node_idx_t>> sc_targets_;
    vec<node_idx_t> contraction_order_;
    vec<vec<cch::packed_shortcut>> cch_sc_up_;
    vec<vec<cch::packed_shortcut>> cch_sc_down_;
    vec<vec<cch::packed_shortcut>> cch_sc_self_;
    vec<vec<cost_t>> cch_cost_up_;
    vec<vec<cost_t>> cch_cost_down_;
    vec<vec<cost_t>> cch_cost_self_;

    vec<pair<node_idx_t, level_bits_t>> multi_level_elevators_;

    vec_map<way_idx_t, component_idx_t> way_component_;
    vec_map<way_idx_t, std::uint8_t> way_importance_;
  };

  cista::wrapped<routing> r_;

  mm_vec_map<node_idx_t, osm_node_idx_t> node_to_osm_;
  mm_vec_map<way_idx_t, osm_way_idx_t> way_osm_idx_;
  mm_vecvec<way_idx_t, point, std::uint64_t> way_polylines_;
  mm_vecvec<way_idx_t, osm_node_idx_t, std::uint64_t> way_osm_nodes_;
  mm_vecvec<string_idx_t, char, std::uint64_t> strings_;
  mm_vec_map<way_idx_t, string_idx_t> way_names_;

  mm_bitvec<way_idx_t> way_has_conditional_access_no_;
  mm_vec<pair<way_idx_t, string_idx_t>> way_conditional_access_no_;

  multi_counter node_way_counter_;
};

}  // namespace osr
