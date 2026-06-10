#pragma once

#include "algorithms/alt/landmarks.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "graph/graph.hpp"

#include <cstddef>
#include <cstdint>
#include <random>
#include <span>
#include <string_view>
#include <vector>

namespace transport {

class AltAlgorithm final : public RoutingAlgorithm {
public:
    explicit AltAlgorithm(const Graph &graph);
    AltAlgorithm(const Graph &graph, uint32_t landmark_count, alt::LandmarkStrategy strategy, uint32_t active_landmarks,
                 std::mt19937 rng, std::span<const NodeCoord> coords = {});

    [[nodiscard]] std::string_view name() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

    [[nodiscard]] uint64_t landmark_table_bytes() const;

private:
    const Graph &graph_;
    uint32_t landmark_count_;
    alt::LandmarkStrategy strategy_;
    uint32_t active_landmarks_;
    std::mt19937 rng_;
    std::span<const NodeCoord> coords_;
    Graph reverse_;
    alt::LandmarkSet landmarks_;
    mutable std::vector<size_t> active_idx_;
    AStarAlgorithm astar_;

    [[nodiscard]] Distance potential(VertexId vertex, VertexId target) const;
    void select_active(VertexId source, VertexId target) const;
};

} // namespace transport
