#pragma once

#include "graph/graph.hpp"

namespace transport {

[[nodiscard]] Graph build_reverse_graph(const Graph &graph);

} // namespace transport
