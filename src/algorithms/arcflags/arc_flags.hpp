#pragma once

#include "algorithms/partition.hpp"
#include "algorithms/phast.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/geometry.hpp"
#include "graph/graph.hpp"
#include "graph/types.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace transport {

struct ArcFlagsPreprocessedData {
    uint16_t regions = 0;
    PartitionMethod partition_method = PartitionMethod::Grid;
    std::vector<uint16_t> region_of;
    std::vector<uint64_t> forward_flags;
};

// Arc-flags shortest-path algorithm.
// graph is the original road graph used for boundary detection, flag computation, and querying.
// preprocess() partitions vertices into regions and computes per-edge uint64_t bitmasks via
// PHAST all-to-one batch sweeps (equality rule) plus an own-region pass.
// At query time, standard Dijkstra skips edges whose bitmask does not include the target's bit.
// Supports up to 64 regions. Multi-threaded flag computation via threads parameter (>= 1).
//
// Requires a prebuilt PhastAlgorithm, moved in via the constructor; this algorithm does not
// build CH/PHAST itself.
class ArcFlagsAlgorithm final : public RoutingAlgorithm {
public:
    explicit ArcFlagsAlgorithm(const Graph &graph, PhastAlgorithm &&phast, uint16_t regions = 32,
                               PartitionMethod partition_method = PartitionMethod::Inertial, uint32_t threads = 1,
                               std::span<const NodeCoord> coords = {});
    explicit ArcFlagsAlgorithm(const Graph &graph, ArcFlagsPreprocessedData &&data);

    std::string_view name() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;
    [[nodiscard]] bool is_preprocessed() const { return preprocessed_; }
    void load_preprocessed(ArcFlagsPreprocessedData &&data);
    [[nodiscard]] ArcFlagsPreprocessedData export_preprocessed() const;

private:
    const Graph &graph_;
    std::optional<PhastAlgorithm> phast_;
    uint16_t regions_;
    PartitionMethod partition_method_;
    uint32_t threads_;
    std::span<const NodeCoord> coords_;
    bool preprocessed_ = false;

    std::vector<uint16_t> region_of_;
    std::vector<uint64_t> forward_flags_;
    mutable StampedVector<Distance> dist_;
};

[[nodiscard]] ArcFlagsPreprocessedData build_arc_flags(const Graph &graph, PhastAlgorithm &phast, uint16_t regions,
                                                       PartitionMethod partition_method, uint32_t threads,
                                                       std::span<const NodeCoord> coords = {});

} // namespace transport
