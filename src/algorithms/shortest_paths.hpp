#pragma once

#include "graph/graph.hpp"

#include <vector>

namespace transport {

void dijkstra_one_to_all(const Graph &graph, VertexId source, std::vector<Distance> &out);

} // namespace transport
