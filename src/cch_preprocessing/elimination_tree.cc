#include "osr/cch_preprocessing/elimination_tree.h"

namespace osr::cch_preprocessing {

void elimination_tree::normal_contraction(graph& adj,
                                          std::vector<std::uint32_t>& res) {
  utl::verify(!adj.empty(), "Adjacency list is empty");

  auto const erase_duplicates = [](auto& values) {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
  };

  std::vector<bool> is_sorted(adj.size(), false);
  for (std::uint32_t i = 0; i < res.size(); ++i) {
    if (adj[i].empty()) {
      continue;
    }
    for (auto const& neighbor : adj[i]) {
      utl::verify(neighbor > i, "Neighbor {} is less than or equal to i {}",
                  neighbor, i);
    }

    if (!is_sorted[i]) {
      erase_duplicates(adj[i]);
      is_sorted[i] = true;
    }
    auto const nxt_min = adj[i][0];
    if (!is_sorted[nxt_min]) {
      erase_duplicates(adj[nxt_min]);
      is_sorted[nxt_min] = true;
    }

    auto new_adj = std::vector<std::uint32_t>{};
    std::set_union(std::next(adj[i].begin()), adj[i].end(),
                   adj[nxt_min].begin(), adj[nxt_min].end(),
                   std::back_inserter(new_adj));
    adj[nxt_min] = std::move(new_adj);
    res[i] = nxt_min;
  }
}

void elimination_tree::fast_contraction(graph const& adj,
                                        std::vector<std::uint32_t>& res) {
  // Initialize DSU
  std::vector<std::uint32_t> parent(res.size());
  std::vector<std::uint32_t> sz(res.size());
  std::vector<std::uint32_t> grp_max(res.size());
  for (std::uint32_t i = 0; i < res.size(); ++i) {
    parent[i] = i;
    sz[i] = 1;
    grp_max[i] = i;
  }

  auto find = [&](std::uint32_t x) {
    std::uint32_t root = x;
    while (root != parent[root]) {
      root = parent[root];
    }
    for (std::uint32_t ptr = x, nxt; ptr != root;) {
      nxt = parent[ptr];
      parent[ptr] = root;
      ptr = nxt;
    }
    return root;
  };

  auto merge = [&](std::uint32_t x, std::uint32_t y) {
    std::uint32_t root_x = find(x);
    std::uint32_t root_y = find(y);
    if (root_x == root_y) {
      return;
    }
    if (sz[root_x] < sz[root_y]) {
      std::swap(root_x, root_y);
    }
    parent[root_y] = root_x;
    sz[root_x] += sz[root_y];
    if (grp_max[root_x] < grp_max[root_y]) {
      std::swap(root_x, root_y);
    }
    res[grp_max[root_y]] = grp_max[root_x];
    grp_max[root_y] = grp_max[root_x];
  };

  // Process each node in increasing order
  for (std::uint32_t u = 0; u < res.size(); ++u) {
    for (auto const& v : adj[u]) {
      merge(u, v);
    }
  }
}

elimination_tree elimination_tree::compute(node_ordering const& ordering,
                                           ways const& w) {
  auto g = graph{ordering.size()};

  // Build the adjacency list for the graph based on the ways and ordering
  ordering.for_each_node<true>(
      [&](node_idx_t const& from_ord, node_idx_t const& from) {
        for (auto const [way, i] : utl::zip_unchecked(
                 w.r_->node_ways_[from], w.r_->node_in_way_idx_[from])) {

          auto const expand = [&](std::uint16_t const to) {
            auto const target_ord =
                ordering.get_ordering(w.r_->way_nodes_[way][to]);
            if (target_ord == from_ord) {
              return;
            }
            if ((kDoFastContraction && target_ord < from_ord) ||
                (!kDoFastContraction && target_ord > from_ord)) {
              g[from_ord.v_].emplace_back(target_ord);
            }
          };

          if (i != 0) {
            expand(i - 1);
          }
          if (i != w.r_->way_nodes_[way].size() - 1U) {
            expand(i + 1);
          }
        }
      });

  auto tmp_res = std::vector<std::uint32_t>(ordering.size(), ordering.size());
  if (kDoFastContraction) {
    fast_contraction(g, tmp_res);  // O(N)
  } else {
    normal_contraction(g, tmp_res);  // O(N^2)
  }

  auto res = vec_map<node_idx_t, node_idx_t>(ordering.size());
  for (std::uint32_t i = 0U; i < ordering.size(); ++i) {
    auto idx = static_cast<node_idx_t>(i);
    res[idx] = (tmp_res[i] == ordering.size())
                   ? node_idx_t::invalid()
                   : static_cast<node_idx_t>(tmp_res[i]);
  }

  return elimination_tree{res};
}

}  // namespace osr::cch_preprocessing