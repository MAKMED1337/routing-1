#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/partition.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"
#include "graph/types.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace transport {

// CHASE: bidirectional CH search with core-subgraph arc-flag pruning.
//
// Preprocessing:
//   1. Build CH. Define core = top core_fraction of CH ranks.
//   2. Extract core-forward and core-backward CSRs from the CH upward graph.
//   3. Partition vertices; propagate region-reachability masks top-down through the core.
//
// Query (two phases):
//   Phase 1 — bidirectional upward CH Dijkstra.  When a vertex in the core is settled it is
//   added to the entry set (fwd or bwd) rather than having its edges relaxed.  Runs to
//   exhaustion so that all core entry-points are collected.
//   Phase 2 — bidirectional Dijkstra on the core subgraphs seeded by the entry sets.
//   Forward arcs are pruned if cf_flags[arc] has no bit in common with target_mask
//   (union of backward-entry reachability); backward arcs are pruned symmetrically.
//   Stopping criterion: both tops exceed mu.
//
// Correctness relies on transitive reachability masks being a superset of true reachability,
// so no shortest-path arc is ever wrongly pruned.
class ChaseAlgorithm final : public RoutingAlgorithm {
public:
    explicit ChaseAlgorithm(const Graph &graph, double core_fraction = 0.05, uint16_t regions = 64,
                            PartitionMethod partition_method = PartitionMethod::Inertial,
                            std::span<const NodeCoord> coords = {});

    std::string_view name() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

private:
    const Graph &graph_;
    uint16_t regions_;
    PartitionMethod partition_method_;
    std::span<const NodeCoord> coords_;

    ContractionHierarchy ch_;
    bool preprocessed_ = false;

    VertexId core_threshold_; // rank >= core_threshold_ => vertex is in core; set at construction

    // Core-forward and core-backward CSRs (upward arcs both in the core).
    std::vector<size_t> cf_offsets_;
    std::vector<Edge> cf_edges_;
    std::vector<size_t> cb_offsets_;
    std::vector<Edge> cb_edges_;

    // Transitive upward-reachability masks computed top-down by rank.
    // cf_reach_[v] = union of region bits reachable upward from v via core-forward arcs.
    std::vector<uint64_t> cf_reach_;
    std::vector<uint64_t> cb_reach_;

    // Per-arc copy of the head's reachability mask (avoids indirect lookup in the hot loop).
    std::vector<uint64_t> cf_flags_;
    std::vector<uint64_t> cb_flags_;

    std::vector<uint16_t> region_of_;

    mutable StampedVector<Distance> fwd_dist_;
    mutable StampedVector<Distance> bwd_dist_;

    void build_core_subgraph();
    void compute_core_flags();
};

} // namespace transport
