#pragma once

#include "algorithms/heuristic.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"
#include "graph/reverse_graph.hpp"

#include <string_view>

namespace transport {

class BidirectionalAStarAlgorithm final : public RoutingAlgorithm {
public:
    BidirectionalAStarAlgorithm(const Graph &graph, Heuristic heuristic);

    [[nodiscard]] std::string_view name() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

private:
    const Graph &graph_;
    Heuristic heuristic_;
    Graph reverse_;
    mutable StampedVector<Distance> forward_dist_;
    mutable StampedVector<Distance> backward_dist_;
};

} // namespace transport
