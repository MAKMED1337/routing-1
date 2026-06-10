#pragma once

#include "graph/types.hpp"

#include <limits>

namespace transport {

constexpr Distance kUnreachable = std::numeric_limits<Distance>::max();

struct QueryStats {
    uint32_t settled = 0;
    uint32_t settled_forward = 0;
    uint32_t settled_backward = 0;
    uint64_t relaxed_arcs = 0;
    uint64_t heap_pushes = 0;
    uint64_t heuristic_evals = 0;
    uint64_t pruned_by_flag = 0;
    uint64_t table_lookups = 0;
    bool used_fallback = false;
};

struct PathResult {
    Distance distance_units = kUnreachable;
    QueryStats stats;
};

} // namespace transport
