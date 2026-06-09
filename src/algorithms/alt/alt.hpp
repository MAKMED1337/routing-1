#pragma once

#include "algorithms/alt/landmarks.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace transport {

class AltAlgorithm final : public RoutingAlgorithm {
public:
    explicit AltAlgorithm(const Graph &graph);
    AltAlgorithm(const Graph &graph, uint32_t landmark_count, alt::LandmarkStrategy strategy, uint32_t active_landmarks,
                 uint32_t seed);

    [[nodiscard]] std::string_view name() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

    [[nodiscard]] uint64_t landmark_table_bytes() const;

private:
    const Graph &graph_;
    uint32_t landmark_count_;
    alt::LandmarkStrategy strategy_;
    uint32_t active_landmarks_;
    uint32_t seed_;
    Graph reverse_;
    alt::LandmarkSet landmarks_;
    mutable StampedVector<Distance> dist_;
    mutable std::vector<size_t> active_idx_;

    [[nodiscard]] int64_t potential(VertexId vertex, VertexId target) const;
    void select_active(VertexId source, VertexId target) const;
};

} // namespace transport
