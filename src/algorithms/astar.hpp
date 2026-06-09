#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"

#include <functional>
#include <string_view>

namespace transport {

class AStarAlgorithm final : public RoutingAlgorithm {
public:
    using Heuristic = std::function<Distance(VertexId, VertexId)>;

    AStarAlgorithm(const Graph &graph, Heuristic heuristic);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

private:
    const Graph &graph_;
    Heuristic heuristic_;
    mutable StampedVector<Distance> g_; // reused query scratch; mutated by the const query()
};

} // namespace transport
