#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"
#include "graph/reverse_graph.hpp"

#include <string_view>

namespace transport {

class BidirectionalDijkstraAlgorithm final : public RoutingAlgorithm {
public:
    explicit BidirectionalDijkstraAlgorithm(const Graph &graph);

    [[nodiscard]] std::string_view name() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

private:
    const Graph &graph_;
    Graph reverse_;
    mutable StampedVector<Distance> forward_dist_;
    mutable StampedVector<Distance> backward_dist_;
};

} // namespace transport
