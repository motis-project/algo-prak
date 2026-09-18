#include "osr/cch_preprocessing/preprocess.h"

namespace osr::cch_preprocessing {

template <Profile P>
auto build_cost_function(ways const& w,
                         node_ordering const& ordering,
                         elimination_tree const& elimination_tree) {
  auto const pp = typename P::parameters{};
  auto cost_function = customized_cost_builder<P>{};

  cost_function.initialize(w.n_nodes());
  return cost_function.build(w, ordering, elimination_tree, pp);
}

void preprocess(fs::path const& in, ways const& w) {

  auto ordering = node_ordering::import(*w.r_);
  utl::verify(ordering.size() == w.n_nodes(),
              "ordering size {} does not match number of nodes {}",
              ordering.size(), w.n_nodes());
  auto elimination_tree = elimination_tree::compute(ordering, w);

  auto build = [&](search_profile const& profile) {
    with_profile(profile, [&]<Profile P>(P&&) {
      auto cost_function =
          build_cost_function<P>(w, ordering, elimination_tree);
      cost_function.write(in);
    });
  };

  // build(search_profile::kFoot);
  build(search_profile::kCar);
  // build(search_profile::kBike);
  // build(search_profile::kBikeElevationLow);
  // build(search_profile::kBikeElevationHigh);

  ordering.write(in);

  fmt::println("Finished CCH preprocessing!");
}

}  // namespace osr::cch_preprocessing