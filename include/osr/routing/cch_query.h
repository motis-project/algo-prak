#pragma once

#include <array>
#include <queue>
#include <stack>

#include "osr/cch_preprocessing/extended_type.h"
#include "osr/cch_preprocessing/preprocessed_data.h"

namespace osr {
namespace prep = cch_preprocessing;

template <Profile P>
struct cch_query {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;

  void initialize() {
    ordering_ = &prep::preprocessed_data::get_ordering();
    cost_function_ = &prep::preprocessed_data::get_customized_cost<P>();
    auto const sz = cost_function_->idx_ranges_.back() +
                    cost_function_->virtual_nodes_.back().size();
    node_costs_.resize(sz);
    is_marked_.resize(sz);
    is_marked_.zero_out();
    for (auto& entry : node_costs_) {
      entry = prep::node_entry::invalid();
    }
  }

  void reset() {
    starts_.clear();
    dests_.clear();
    for (auto const& idx : traversed_nodes_) {
      node_costs_[idx] = prep::node_entry::invalid();
    }
    traversed_nodes_.clear();
  }

  prep::node_entry const get_node_entry(node const n) const {
    auto const idx = cost_function_->get_flatten_node_idx(
        cost_function_->get_virtual_node_idx(n, *ordering_));
    if (node_costs_[idx] != prep::node_entry::invalid()) {
      return node_costs_.at(idx);
    }
    return prep::node_entry::invalid();
  }

  prep::node_entry const get_node_entry(prep::ext_node const n) const {
    auto const idx = cost_function_->get_flatten_node_idx(n);
    if (node_costs_[idx] != prep::node_entry::invalid()) {
      return node_costs_.at(idx);
    }
    return prep::node_entry::invalid();
  }

  void add_start(node const n, cost_t const start_cost) {
    auto const start_idx = cost_function_->get_virtual_node_idx(n, *ordering_);
    if (start_idx != prep::ext_node::invalid()) {
      starts_.emplace_back(cost_function_->get_flatten_node_idx(start_idx));
      auto content = prep::node_entry::invalid();
      content.cost_ = start_cost;
      node_costs_[starts_.back()] = content;
    }
  }

  void add_dest(node const n) {
    auto const dest_idx = cost_function_->get_virtual_node_idx(n, *ordering_);
    if (dest_idx != prep::ext_node::invalid()) {
      dests_.emplace_back(cost_function_->get_flatten_node_idx(dest_idx));
    }
  }

  cost_t get_best_cost() const {
    cost_t best_cost = kInfeasible;
    for (auto const& dest : dests_) {
      auto const entry = node_costs_[dest];
      best_cost = std::min(best_cost, entry.cost_);
    }
    return best_cost;
  }

  bool run() {
    std::stack<std::uint32_t> st{};
    std::list<std::uint32_t> prep_order{};
    for (bool phase : {true, false}) {
      for (auto const& idx : phase ? starts_ : dests_) {
        if (is_marked_.test(idx)) {
          continue;
        }
        st.push(idx);
        for (auto u = idx; true;) {
          is_marked_.set(u);
          auto const& par = cost_function_->get_parent(u);
          if (par != node_idx_t::invalid() && !is_marked_.test(par.v_)) {
            u = par.v_;
            st.push(u);
          } else {
            break;
          }
        }
        while (!st.empty()) {
          auto const u = st.top();
          st.pop();
          prep_order.emplace_back(u);
        }
      }

      auto const it_begin = !phase ? prep_order.begin() : prep_order.end();
      auto const it_end = !phase ? prep_order.end() : prep_order.begin();
      for (auto it = it_begin; it != it_end; !phase ? ++it : --it) {
        auto const u = !phase ? *it : *std::prev(it);
        is_marked_.set(u, false);
        traversed_nodes_.emplace_back(u);
        if (node_costs_[u] == prep::node_entry::invalid()) {
          continue;
        }
        auto const curr_cost = node_costs_[u].cost_;
        auto const f = [&](auto const& uv_idx, auto const& uv) {
          if (curr_cost >= kInfeasible - uv.cost_) {
            return;
          }
          auto const v_idx = cost_function_->get_flatten_node_idx(uv.to_);
          auto const new_cost = curr_cost + uv.cost_;

          traversed_nodes_.emplace_back(v_idx);
          node_costs_[v_idx].update(new_cost,
                                    static_cast<prep::ext_edge_idx_t>(uv_idx));
        };
        if (phase) {
          cost_function_->template for_each_edge<true>(u, f);
        } else {
          cost_function_->template for_each_edge<false>(u, f);
        }
      }
      prep_order.clear();
    }

    return true;
  }

  vec<prep::node_entry> node_costs_;
  bitvec<std::uint32_t> is_marked_;
  std::vector<std::uint32_t> starts_, dests_;
  std::vector<std::uint32_t> traversed_nodes_;

  prep::customized_cost_stored<P> const* cost_function_{};
  prep::node_ordering const* ordering_{};
};
}  // namespace osr