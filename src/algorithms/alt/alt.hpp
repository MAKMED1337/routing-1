#pragma once

#include "algorithms/alt/landmarks.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "graph/graph.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace transport {

class AltAlgorithm final : public RoutingAlgorithm {
public:
    explicit AltAlgorithm(const Graph &graph);
    AltAlgorithm(const Graph &graph, alt::LandmarkSet landmarks, uint32_t active_landmarks);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

    [[nodiscard]] uint64_t landmark_table_bytes() const;

private:
    const Graph &graph_;
    uint32_t active_landmarks_;
    alt::LandmarkSet landmarks_;
    mutable std::vector<size_t> active_idx_;
    AStarAlgorithm astar_;

    [[nodiscard]] Distance potential(VertexId vertex, VertexId target) const;
    void select_active(VertexId source, VertexId target) const;
};

} // namespace transport
