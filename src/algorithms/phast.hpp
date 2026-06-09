#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "graph/types.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace transport {

// Precompute the inverse rank array (inv_rank[r] = vertex with rank r) needed by the legacy APIs.
[[nodiscard]] std::vector<VertexId> build_inv_rank(const ContractionHierarchy &ch);

// Legacy APIs: operate in original vertex-id space; require a pre-built inv_rank.
void phast_all_to_one(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId target,
                      std::vector<Distance> &dist);
void phast_one_to_all(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId source,
                      std::vector<Distance> &dist);

// Rank-ordered cache layout for optimized PHAST queries.
// Stores rank-indexed upward CSRs with edge targets as ranks instead of original vertex ids,
// enabling cache-friendly sweeps without per-query translation.
struct PhastContext {
    std::vector<VertexId> rank_to_vertex; // rank  → original vertex
    std::vector<VertexId> vertex_to_rank; // vertex → rank

    // Rank-indexed upward CSR; each edge's .to field holds the destination rank.
    std::vector<uint64_t> fwd_offsets;
    std::vector<Edge> fwd_edges;
    std::vector<uint64_t> bwd_offsets;
    std::vector<Edge> bwd_edges;

    explicit PhastContext(const ContractionHierarchy &ch);

    [[nodiscard]] VertexId vertex_count() const;
    [[nodiscard]] std::span<const Edge> fwd_adjacent_edges(VertexId rank) const;
    [[nodiscard]] std::span<const Edge> bwd_adjacent_edges(VertexId rank) const;
};

// Optimized single-target APIs: work in rank-space, resize dist to vertex_count().
void phast_all_to_one(const PhastContext &ctx, VertexId target, std::vector<Distance> &dist);
void phast_one_to_all(const PhastContext &ctx, VertexId source, std::vector<Distance> &dist);

// Batched all-to-one for an arbitrary number of targets.
// Output layout (vertex-major, original-id order): dist[v * targets.size() + i] == d(v, targets[i]).
// dist is resized to vertex_count() * targets.size().
void phast_all_to_one_batch(const PhastContext &ctx, std::span<const VertexId> targets, std::vector<Distance> &dist);

} // namespace transport
