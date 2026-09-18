#pragma once

#include <string_view>

#include "osr/routing/profile.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/ferry.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/profiles/railway.h"
#include "osr/types.h"

namespace osr::cch_preprocessing {

template <Profile P>
struct profile_name;

template <>
struct profile_name<foot<false, noop_tracking>> {
  static constexpr std::string_view const value = "cc_foot_noop_tracking.bin";
};
template <>
struct profile_name<foot<true, noop_tracking>> {
  static constexpr std::string_view const value =
      "cc_wheelchair_noop_tracking.bin";
};
template <>
struct profile_name<foot<false, elevator_tracking>> {
  static constexpr std::string_view const value = "cc_foot.bin";
};
template <>
struct profile_name<foot<true, elevator_tracking>> {
  static constexpr std::string_view const value = "cc_wheelchair.bin";
};
template <>
struct profile_name<bike<bike_costing::kSafe, kElevationNoCost>> {
  static constexpr std::string_view const value = "cc_bike.bin";
};
template <>
struct profile_name<bike<bike_costing::kFast, kElevationNoCost>> {
  static constexpr std::string_view const value = "cc_bike_fast.bin";
};
template <>
struct profile_name<bike<bike_costing::kSafe, kElevationLowCost>> {
  static constexpr std::string_view const value = "cc_bike_elevation_low.bin";
};
template <>
struct profile_name<bike<bike_costing::kSafe, kElevationHighCost>> {
  static constexpr std::string_view const value = "cc_bike_elevation_high.bin";
};
template <>
struct profile_name<car> {
  static constexpr std::string_view const value = "cc_car.bin";
};
template <>
struct profile_name<car_parking<false, false>> {
  static constexpr std::string_view const value = "cc_car_dropoff.bin";
};
template <>
struct profile_name<car_parking<true, false>> {
  static constexpr std::string_view const value =
      "cc_car_dropoff_wheelchair.bin";
};
template <>
struct profile_name<car_parking<false, true>> {
  static constexpr std::string_view const value = "cc_car_parking.bin";
};
template <>
struct profile_name<car_parking<true, true>> {
  static constexpr std::string_view const value =
      "cc_car_parking_wheelchair.bin";
};
template <>
struct profile_name<bike_sharing> {
  static constexpr std::string_view const value = "cc_bike_sharing.bin";
};
template <>
struct profile_name<car_sharing<track_node_tracking>> {
  static constexpr std::string_view const value = "cc_car_sharing.bin";
};
template <>
struct profile_name<bus> {
  static constexpr std::string_view const value = "cc_bus.bin";
};
template <>
struct profile_name<railway> {
  static constexpr std::string_view const value = "cc_railway.bin";
};
template <>
struct profile_name<ferry> {
  static constexpr std::string_view const value = "cc_ferry.bin";
};

}  // namespace osr::cch_preprocessing