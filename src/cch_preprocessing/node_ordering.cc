#include "osr/cch_preprocessing/node_ordering.h"

#include "fmt/core.h"
#include "fmt/std.h"

namespace osr::cch_preprocessing {

node_ordering node_ordering::randomize(std::uint32_t num_nodes,
                                       std::uint32_t seed) {
  node_ordering ordering;
  ordering.old_to_new_.resize(num_nodes);
  ordering.new_to_old_.resize(num_nodes);
  vec<node_idx_t> nodes(num_nodes);
  std::iota(begin(nodes), end(nodes), static_cast<node_idx_t>(0U));
  std::mt19937 rng(seed);
  std::shuffle(begin(nodes), end(nodes), rng);
  for (std::uint32_t i = 0U; i < nodes.size(); ++i) {
    auto const node = nodes[i];
    ordering.old_to_new_[node] = static_cast<node_idx_t>(i);
    ordering.new_to_old_[static_cast<node_idx_t>(i)] = node;
  }
  return ordering;
}
}  // namespace osr::cch_preprocessing