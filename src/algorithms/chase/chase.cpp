#include "algorithms/chase/chase.hpp"

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/heap_node.hpp"
#include "algorithms/partition.hpp"
#include "graph/graph.hpp"
#include "graph/types.hpp"
#include "routing/routing.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace transport {

ChaseAlgorithm::ChaseAlgorithm(const Graph &graph, double core_fraction, uint16_t regions,
                               PartitionMethod partition_method, std::span<const NodeCoord> coords)
    : graph_(graph), regions_(regions), partition_method_(partition_method), coords_(coords), core_threshold_(0),
      fwd_dist_(graph.vertex_count(), kUnreachable), bwd_dist_(graph.vertex_count(), kUnreachable) {
    if (regions == 0 || regions > 64) {
        throw std::invalid_argument("chase: regions must be in [1, 64]");
    }
    if (core_fraction <= 0.0 || core_fraction > 1.0 || !std::isfinite(core_fraction)) {
        throw std::invalid_argument("chase: core_fraction must be in (0, 1]");
    }
    if (partition_method_ == PartitionMethod::Inertial && !std::has_single_bit(regions)) {
        throw std::invalid_argument("chase: inertial partition requires regions to be a power of two");
    }
    core_threshold_ = static_cast<VertexId>(static_cast<double>(graph.vertex_count()) * (1.0 - core_fraction) + 0.5);
}

std::string_view ChaseAlgorithm::name() const { return "chase"; }

void ChaseAlgorithm::preprocess() {
    if (preprocessed_) {
        return;
    }

    ContractionHierarchyAlgorithm ch_algo(graph_);
    ch_algo.preprocess();
    ch_ = ch_algo.get_ch();

    region_of_ = build_partition(graph_, regions_, partition_method_, coords_);

    build_core_subgraph();
    compute_core_flags();

    preprocessed_ = true;
}

void ChaseAlgorithm::build_core_subgraph() {
    const VertexId V = ch_.vertex_count();
    cf_offsets_.assign(V + 1, 0);
    cb_offsets_.assign(V + 1, 0);

    for (VertexId u = 0; u < V; ++u) {
        if (ch_.rank[u] < core_threshold_) {
            continue;
        }
        for (const Edge &e : ch_.forward_adjacent_edges(u)) {
            if (ch_.rank[e.to] >= core_threshold_) {
                ++cf_offsets_[u + 1];
            }
        }
        for (const Edge &e : ch_.backward_adjacent_edges(u)) {
            if (ch_.rank[e.to] >= core_threshold_) {
                ++cb_offsets_[u + 1];
            }
        }
    }

    for (VertexId u = 0; u < V; ++u) {
        cf_offsets_[u + 1] += cf_offsets_[u];
        cb_offsets_[u + 1] += cb_offsets_[u];
    }

    cf_edges_.resize(cf_offsets_[V]);
    cb_edges_.resize(cb_offsets_[V]);

    std::vector<size_t> cf_fill(cf_offsets_.begin(), cf_offsets_.end());
    std::vector<size_t> cb_fill(cb_offsets_.begin(), cb_offsets_.end());

    for (VertexId u = 0; u < V; ++u) {
        if (ch_.rank[u] < core_threshold_) {
            continue;
        }
        for (const Edge &e : ch_.forward_adjacent_edges(u)) {
            if (ch_.rank[e.to] >= core_threshold_) {
                cf_edges_[cf_fill[u]++] = e;
            }
        }
        for (const Edge &e : ch_.backward_adjacent_edges(u)) {
            if (ch_.rank[e.to] >= core_threshold_) {
                cb_edges_[cb_fill[u]++] = e;
            }
        }
    }
}

void ChaseAlgorithm::compute_core_flags() {
    const VertexId V = ch_.vertex_count();

    cf_reach_.assign(V, 0);
    cb_reach_.assign(V, 0);

    std::vector<VertexId> by_rank(V);
    std::iota(by_rank.begin(), by_rank.end(), VertexId{0});
    std::sort(by_rank.begin(), by_rank.end(), [&](VertexId a, VertexId b) { return ch_.rank[a] > ch_.rank[b]; });

    for (const VertexId v : by_rank) {
        if (ch_.rank[v] < core_threshold_) {
            continue;
        }
        uint64_t cf_mask = 1ULL << region_of_[v];
        uint64_t cb_mask = 1ULL << region_of_[v];
        for (size_t k = cf_offsets_[v]; k < cf_offsets_[v + 1]; ++k) {
            cf_mask |= cf_reach_[cf_edges_[k].to];
        }
        for (size_t k = cb_offsets_[v]; k < cb_offsets_[v + 1]; ++k) {
            cb_mask |= cb_reach_[cb_edges_[k].to];
        }
        cf_reach_[v] = cf_mask;
        cb_reach_[v] = cb_mask;
    }

    cf_flags_.assign(cf_edges_.size(), 0);
    cb_flags_.assign(cb_edges_.size(), 0);
    for (VertexId v = 0; v < V; ++v) {
        for (size_t k = cf_offsets_[v]; k < cf_offsets_[v + 1]; ++k) {
            cf_flags_[k] = cf_reach_[cf_edges_[k].to];
        }
        for (size_t k = cb_offsets_[v]; k < cb_offsets_[v + 1]; ++k) {
            cb_flags_[k] = cb_reach_[cb_edges_[k].to];
        }
    }
}

PathResult ChaseAlgorithm::query(VertexId source, VertexId target) const {
    if (!preprocessed_) {
        throw std::runtime_error("chase: preprocess() must be called before query()");
    }

    fwd_dist_.reset();
    bwd_dist_.reset();
    fwd_dist_.set(source, 0);
    bwd_dist_.set(target, 0);

    HeapQueue fwd_pq;
    HeapQueue bwd_pq;
    fwd_pq.push({0, source});
    bwd_pq.push({0, target});

    std::vector<std::pair<VertexId, Distance>> fwd_entries;
    std::vector<std::pair<VertexId, Distance>> bwd_entries;

    Distance mu = kUnreachable;
    uint32_t settled = 0;

    // Phase 1: bidirectional CH upward search. Core vertices are collected into entry sets
    // without relaxing their edges; non-core vertices are settled normally.
    // Runs to exhaustion so all first-contact core vertices are found regardless of mu.
    auto p1_settle = [&](bool is_fwd) {
        auto &my_pq = is_fwd ? fwd_pq : bwd_pq;
        auto &my_dist = is_fwd ? fwd_dist_ : bwd_dist_;
        auto &opp_dist = is_fwd ? bwd_dist_ : fwd_dist_;
        auto &entries = is_fwd ? fwd_entries : bwd_entries;

        const HeapNode top = my_pq.top();
        my_pq.pop();
        if (top.key != my_dist.get(top.v)) {
            return;
        }
        ++settled;

        const Distance opp = opp_dist.get(top.v);
        if (opp != kUnreachable) {
            mu = std::min(mu, top.key + opp);
        }

        if (ch_.rank[top.v] >= core_threshold_) {
            entries.push_back({top.v, top.key});
            return;
        }

        const auto adj = is_fwd ? ch_.forward_adjacent_edges(top.v) : ch_.backward_adjacent_edges(top.v);
        for (const Edge &e : adj) {
            const Distance nd = top.key + static_cast<Distance>(e.weight);
            if (nd < mu && nd < my_dist.get(e.to)) {
                my_dist.set(e.to, nd);
                my_pq.push({nd, e.to});
            }
        }
    };

    while (!fwd_pq.empty() || !bwd_pq.empty()) {
        const Distance ftop = fwd_pq.empty() ? kUnreachable : fwd_pq.top().key;
        const Distance btop = bwd_pq.empty() ? kUnreachable : bwd_pq.top().key;
        if (!fwd_pq.empty() && (bwd_pq.empty() || ftop <= btop)) {
            p1_settle(true);
        } else {
            p1_settle(false);
        }
    }

    // Phase 2: bidirectional search on the core subgraphs.
    // Arc (u→v) is pruned if its flags share no bit with the opposite side's mask.
    if (!fwd_entries.empty() && !bwd_entries.empty()) {
        uint64_t target_mask = 0;
        uint64_t source_mask = 0;
        for (const auto &[v, d] : bwd_entries) {
            target_mask |= cb_reach_[v];
        }
        for (const auto &[v, d] : fwd_entries) {
            source_mask |= cf_reach_[v];
        }

        HeapQueue cf_pq;
        HeapQueue cb_pq;

        for (const auto &[v, d] : fwd_entries) {
            if (d < fwd_dist_.get(v)) {
                fwd_dist_.set(v, d);
            }
            if (d < mu) {
                cf_pq.push({d, v});
            }
        }
        for (const auto &[v, d] : bwd_entries) {
            if (d < bwd_dist_.get(v)) {
                bwd_dist_.set(v, d);
            }
            if (d < mu) {
                cb_pq.push({d, v});
            }
        }

        auto p2_settle = [&](bool is_fwd) {
            auto &my_pq = is_fwd ? cf_pq : cb_pq;
            const auto &my_off = is_fwd ? cf_offsets_ : cb_offsets_;
            const auto &my_edges = is_fwd ? cf_edges_ : cb_edges_;
            const auto &my_flags = is_fwd ? cf_flags_ : cb_flags_;
            auto &my_dist = is_fwd ? fwd_dist_ : bwd_dist_;
            auto &opp_dist = is_fwd ? bwd_dist_ : fwd_dist_;
            const uint64_t prune_mask = is_fwd ? target_mask : source_mask;

            const HeapNode top = my_pq.top();
            my_pq.pop();
            if (top.key != my_dist.get(top.v)) {
                return;
            }
            ++settled;

            const Distance opp = opp_dist.get(top.v);
            if (opp != kUnreachable) {
                const Distance cand = top.key + opp;
                if (cand < mu) {
                    mu = cand;
                }
            }

            for (size_t k = my_off[top.v]; k < my_off[top.v + 1]; ++k) {
                if (!(my_flags[k] & prune_mask)) {
                    continue;
                }
                const Edge &e = my_edges[k];
                const Distance nd = top.key + static_cast<Distance>(e.weight);
                if (nd < mu && nd < my_dist.get(e.to)) {
                    my_dist.set(e.to, nd);
                    my_pq.push({nd, e.to});
                }
            }
        };

        while (!cf_pq.empty() || !cb_pq.empty()) {
            const Distance ftop = cf_pq.empty() ? kUnreachable : cf_pq.top().key;
            const Distance btop = cb_pq.empty() ? kUnreachable : cb_pq.top().key;
            if (ftop >= mu && btop >= mu) {
                break;
            }
            if (!cf_pq.empty() && (cb_pq.empty() || ftop <= btop)) {
                p2_settle(true);
            } else {
                p2_settle(false);
            }
        }
    }

    return PathResult{mu, settled};
}

} // namespace transport
