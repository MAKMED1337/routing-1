#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "graph/types.hpp"

#include <vector>

namespace transport {

// All-to-one: compute d(v, target) for every vertex v. dist must have size vertex_count.
// Uses backward upward Dijkstra from target + forward downward sweep.
void phast_all_to_one(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId target,
                      std::vector<Distance> &dist);

// One-to-all: compute d(source, v) for every vertex v. dist must have size vertex_count.
// Uses forward upward Dijkstra from source + backward downward sweep.
void phast_one_to_all(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId source,
                      std::vector<Distance> &dist);

// Precompute the inverse rank array (inv_rank[r] = vertex with rank r) needed by PHAST.
[[nodiscard]] std::vector<VertexId> build_inv_rank(const ContractionHierarchy &ch);

} // namespace transport
