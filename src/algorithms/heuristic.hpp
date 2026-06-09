#pragma once

#include "graph/types.hpp"

#include <functional>

namespace transport {

using Heuristic = std::function<Distance(VertexId, VertexId)>;

} // namespace transport
