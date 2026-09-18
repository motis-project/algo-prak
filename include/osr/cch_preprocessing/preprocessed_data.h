#pragma once

#include <filesystem>
#include <variant>

#include "cista/memory_holder.h"

#include "osr/cch_preprocessing/customized_cost.h"
#include "osr/cch_preprocessing/elimination_tree.h"
#include "osr/cch_preprocessing/node_ordering.h"
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

using customized_costs = std::variant<
    bool,
    cista::wrapped<customized_cost_stored<foot<false, noop_tracking>>>,
    cista::wrapped<customized_cost_stored<foot<true, noop_tracking>>>,
    cista::wrapped<customized_cost_stored<foot<false, elevator_tracking>>>,
    cista::wrapped<customized_cost_stored<foot<true, elevator_tracking>>>,
    cista::wrapped<
        customized_cost_stored<bike<bike_costing::kSafe, kElevationNoCost>>>,
    cista::wrapped<
        customized_cost_stored<bike<bike_costing::kFast, kElevationNoCost>>>,
    cista::wrapped<
        customized_cost_stored<bike<bike_costing::kSafe, kElevationLowCost>>>,
    cista::wrapped<
        customized_cost_stored<bike<bike_costing::kSafe, kElevationHighCost>>>,
    cista::wrapped<customized_cost_stored<car>>,
    cista::wrapped<customized_cost_stored<car_parking<false, false>>>,
    cista::wrapped<customized_cost_stored<car_parking<true, false>>>,
    cista::wrapped<customized_cost_stored<car_parking<false, true>>>,
    cista::wrapped<customized_cost_stored<car_parking<true, true>>>,
    cista::wrapped<customized_cost_stored<bike_sharing>>,
    cista::wrapped<customized_cost_stored<car_sharing<track_node_tracking>>>,
    cista::wrapped<customized_cost_stored<bus>>,
    cista::wrapped<customized_cost_stored<railway>>,
    cista::wrapped<customized_cost_stored<ferry>>>;

struct preprocessed_data {

  static void load_metric_independent(std::filesystem::path const& dir) {
    utl::verify(std::filesystem::exists(dir),
                "preprocessed data directory does not exist");
    ordering_ = node_ordering::read(dir);
  }

  template <Profile P, bool relaxUpdate = false>
  static void load_customized_cost(std::filesystem::path const& dir) {
    utl::verify(std::filesystem::exists(dir),
                "preprocessed data directory does not exist");
    if (relaxUpdate &&
        std::holds_alternative<cista::wrapped<customized_cost_stored<P>>>(
            customized_cost_)) {
      return;
    }
    customized_cost_ = customized_cost_stored<P>::read(dir);
  }

  template <Profile P>
  static customized_cost_stored<P> const& get_customized_cost() {
    utl::verify(
        std::holds_alternative<cista::wrapped<customized_cost_stored<P>>>(
            customized_cost_),
        "customized cost for profile {} not loaded", profile_name<P>::value);
    return *std::get<cista::wrapped<customized_cost_stored<P>>>(
        customized_cost_);
  }

  static node_ordering const& get_ordering() {
    utl::verify(ordering_, "ordering not loaded");
    return *ordering_;
  }

  inline static cista::wrapped<node_ordering> ordering_ =
      cista::wrapped<node_ordering>{cista::raw::make_unique<node_ordering>()};
  inline static customized_costs customized_cost_ =
      false;  // default to false, meaning no customized cost is loaded
};

}  // namespace osr::cch_preprocessing