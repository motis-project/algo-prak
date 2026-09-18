#pragma once

#include <filesystem>

#include "utl/progress_tracker.h"

#include "osr/cch_preprocessing/customized_cost.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/with_profile.h"

namespace osr::cch_preprocessing {

namespace fs = std::filesystem;

void preprocess(fs::path const&, ways const&);

}  // namespace osr::cch_preprocessing