#pragma once

#include <filesystem>
#include <memory>
#include <optional>

#include "osr/lookup.h"
#include "osr/routing/parameters.h"
#include "osr/routing/path.h"
#include "osr/routing/profile.h"
#include "osr/ways.h"

namespace osr {
struct elevation_storage;
struct sharing_data;
}  // namespace osr

namespace osr::cch {

struct cache {
  cache();
  ~cache();
  struct impl;
  std::unique_ptr<impl> impl_;
};

constexpr auto const kFileName = "cch.bin";

void preprocess(ways::routing const&, std::filesystem::path const& out);

bool usable(ways const&,
            search_profile,
            bitvec<node_idx_t> const* blocked,
            sharing_data const* sharing,
            elevation_storage const* elevations);

std::optional<path> route(profile_parameters const&,
                          ways const&,
                          lookup const&,
                          search_profile,
                          location const& from,
                          location const& to,
                          match_view_t from_match,
                          match_view_t to_match,
                          cost_t max,
                          direction);

}  // namespace osr::cch
