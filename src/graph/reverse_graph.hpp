#pragma once

#include "graph/graph.hpp"

#include <vector>

namespace transport {

struct ReverseGraph {
    std::vector<uint64_t> offsets;
    std::vector<Edge> edges;
};

[[nodiscard]] ReverseGraph build_reverse_graph(const Graph &graph);

} // namespace transport
