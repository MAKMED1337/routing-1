#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"

#include <chrono>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace transport {

struct PreprocessStats {
    std::chrono::nanoseconds ordering_init{0};
    uint64_t witness_calls = 0;
};

class ContractionHierarchy {
public:
    std::vector<uint32_t> rank;
    std::vector<uint64_t> forward_offsets;
    std::vector<Edge> forward_edges;
    std::vector<uint64_t> backward_offsets;
    std::vector<Edge> backward_edges;

    [[nodiscard]] VertexId vertex_count() const;
    [[nodiscard]] std::span<const Edge> forward_adjacent_edges(VertexId vertex) const;
    [[nodiscard]] std::span<const Edge> backward_adjacent_edges(VertexId vertex) const;
};

struct ContractionHierarchyBuildResult {
    ContractionHierarchy hierarchy;
    PreprocessStats stats;
};

// Builds a contraction hierarchy from `graph`. This is the reusable entry point for sharing a
// CH across multiple CH-dependent algorithms; ContractionHierarchyAlgorithm::preprocess() uses it.
[[nodiscard]] ContractionHierarchyBuildResult build_contraction_hierarchy(const Graph &graph);

class ContractionHierarchyAlgorithm final : public RoutingAlgorithm {
public:
    explicit ContractionHierarchyAlgorithm(const Graph &graph);

    [[nodiscard]] std::string_view name() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;
    [[nodiscard]] uint64_t auxiliary_edge_count() const;
    [[nodiscard]] PreprocessStats preprocess_stats() const { return last_stats_; }
    [[nodiscard]] const ContractionHierarchy &get_ch() const;

private:
    const Graph &graph_;
    ContractionHierarchy ch_;
    bool preprocessed_ = false;
    PreprocessStats last_stats_;

    // Reused bidirectional-query scratch (one per search direction); mutated by the const query().
    mutable StampedVector<Distance> forward_dist_;
    mutable StampedVector<Distance> backward_dist_;
};

} // namespace transport
