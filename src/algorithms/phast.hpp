#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "graph/types.hpp"

#include <span>
#include <vector>

namespace transport {

// Rank-ordered cache layout for PHAST queries.
// Stores rank-indexed upward CSRs with edge targets as ranks instead of original vertex ids,
// enabling cache-friendly sweeps without per-query translation.
struct PhastAlgorithm {
    std::vector<VertexId> rank_to_vertex; // rank  → original vertex
    std::vector<VertexId> vertex_to_rank; // vertex → rank

    // Rank-indexed upward CSR; each edge's .to field holds the destination rank.
    std::vector<size_t> fwd_offsets;
    std::vector<Edge> fwd_edges;
    std::vector<size_t> bwd_offsets;
    std::vector<Edge> bwd_edges;

    explicit PhastAlgorithm(const ContractionHierarchy &ch);

    [[nodiscard]] VertexId vertex_count() const;
    [[nodiscard]] std::span<const Edge> fwd_adjacent_edges(VertexId rank) const;
    [[nodiscard]] std::span<const Edge> bwd_adjacent_edges(VertexId rank) const;

    // dist[v] == d(v, target) for all v; dist is resized to vertex_count().
    void all_to_one(VertexId target, std::vector<Distance> &dist) const;
    // dist[v] == d(source, v) for all v; dist is resized to vertex_count().
    void one_to_all(VertexId source, std::vector<Distance> &dist) const;
    // dist[v * B + i] == d(v, targets[i]); dist is resized to vertex_count() * B.
    void all_to_one_batch(std::span<const VertexId> targets, std::vector<Distance> &dist) const;
};

} // namespace transport
