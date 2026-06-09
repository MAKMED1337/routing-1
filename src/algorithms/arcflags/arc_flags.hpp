#pragma once

#include "algorithms/partition.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"
#include "graph/types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace transport {

struct PhastAlgorithm;

// Arc-flags shortest-path algorithm.
// Preprocessing: builds a CH, partitions vertices into regions, then computes per-edge uint64_t
// bitmasks via PHAST all-to-one batch sweeps from each region's boundary vertices (equality rule)
// plus an own-region rule pass. At query time, standard Dijkstra skips any edge whose flag
// bitmask does not include the target region's bit.
// Supports up to 64 regions. Multi-threaded flag computation via threads parameter.
class ArcFlagsAlgorithm final : public RoutingAlgorithm {
public:
    explicit ArcFlagsAlgorithm(const Graph &graph, uint32_t regions = 32, std::string partition_method = "inertial",
                               uint32_t threads = 1);

    std::string_view name() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

private:
    const Graph &graph_;
    uint32_t regions_;
    PartitionMethod partition_method_;
    uint32_t threads_;
    bool preprocessed_ = false;

    std::vector<uint16_t> region_of_;
    std::vector<uint64_t> forward_flags_;
    mutable StampedVector<Distance> dist_;

    void compute_flags(const PhastAlgorithm &phast, const std::vector<std::vector<VertexId>> &boundary_by_region);
};

} // namespace transport
