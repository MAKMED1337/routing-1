#pragma once

#include "graph/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace transport {

constexpr uint32_t kDistanceScale = 1000;

struct Edge {
    VertexId to = 0;
    Weight weight = 0;
};

class Graph {
public:
    VertexId vertex_count_ = 0;
    std::vector<size_t> offsets;
    std::vector<Edge> edges;

    [[nodiscard]] VertexId vertex_count() const;
    [[nodiscard]] uint64_t edge_count() const;
    [[nodiscard]] std::span<const Edge> adjacent_edges(VertexId vertex) const;
};

} // namespace transport
