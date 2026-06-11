#pragma once

#include "algorithms/alt/landmarks.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "graph/graph.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace transport {

class AltAlgorithm final : public RoutingAlgorithm {
public:
    // Deferred: calls preprocess() to build 16 farthest-point landmarks.
    explicit AltAlgorithm(const Graph &graph);
    // Deferred: calls preprocess() to build configurable landmarks.
    AltAlgorithm(const Graph &graph, alt::LandmarkStrategy landmark_strategy, uint32_t landmark_count,
                 uint32_t active_landmarks, std::span<const NodeCoord> coords = {});
    // Immediate: landmarks are already built; preprocess() is a no-op.
    AltAlgorithm(const Graph &graph, alt::LandmarkSet landmarks, uint32_t active_landmarks);

    [[nodiscard]] std::string_view name() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

    [[nodiscard]] uint64_t landmark_table_bytes() const;

private:
    const Graph &graph_;
    uint32_t active_landmarks_;
    alt::LandmarkStrategy landmark_strategy_;
    uint32_t landmark_count_;
    std::vector<NodeCoord> landmark_coords_;
    alt::LandmarkSet landmarks_;
    bool preprocessed_;
    mutable std::vector<size_t> active_idx_;
    AStarAlgorithm astar_;

    [[nodiscard]] Distance potential(VertexId vertex, VertexId target) const;
    void select_active(VertexId source, VertexId target) const;
};

} // namespace transport
