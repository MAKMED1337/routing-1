#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "algorithms/routing_preprocessing_context.hpp"
#include "graph/geometry.hpp"
#include "graph/graph.hpp"

#include <memory>
#include <span>
#include <string>

namespace transport {

// Owns a graph-bound preprocessing context plus the constructed RoutingAlgorithm.
// make_routing_instance() construction is cheap: it validates arguments but builds no expensive
// artifacts. preprocess() sequences dependency-artifact construction (CH/PHAST, timed
// separately as "dependency" cost) before constructing and preprocessing the algorithm itself
// (timed as "algorithm" cost), so benchmarks can attribute shared preprocessing correctly.
class RoutingInstance {
public:
    RoutingInstance(const Graph &graph, std::string algorithm_name, std::span<const NodeCoord> coords);

    void preprocess();

    [[nodiscard]] RoutingAlgorithm &algorithm();
    [[nodiscard]] const RoutingAlgorithm &algorithm() const;

    [[nodiscard]] const RoutingPreprocessingContext &context() const { return context_; }

    struct PreprocessTiming {
        double dependency_wall_s = 0.0;
        double dependency_cpu_s = 0.0;
        double algorithm_wall_s = 0.0;
        double algorithm_cpu_s = 0.0;
        double after_dependency_peak_rss_mb = 0.0;
        double after_algorithm_peak_rss_mb = 0.0;
    };
    [[nodiscard]] const PreprocessTiming &timing() const { return timing_; }

private:
    const Graph &graph_;
    std::string algorithm_name_;
    std::span<const NodeCoord> coords_;
    RoutingPreprocessingContext context_;
    std::unique_ptr<RoutingAlgorithm> algorithm_;
    bool preprocessed_ = false;
    PreprocessTiming timing_;

    [[nodiscard]] std::unique_ptr<RoutingAlgorithm> build_algorithm();
};

// Validates `name`/`coords` and returns a cheaply-constructed RoutingInstance. Call
// instance.preprocess() before instance.algorithm().query(...).
[[nodiscard]] RoutingInstance make_routing_instance(const std::string &name, const Graph &graph,
                                                    std::span<const NodeCoord> coords = {});

} // namespace transport
