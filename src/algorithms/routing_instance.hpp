#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "graph/geometry.hpp"
#include "graph/graph.hpp"

#include <chrono>
#include <memory>
#include <optional>
#include <span>
#include <string>

namespace transport {

struct DependencyPreprocessStats {
    std::chrono::nanoseconds wall{0};
    std::chrono::nanoseconds cpu{0};
    // Process-wide high-water RSS sampled right after this phase, not memory owned by this
    // dependency alone. In make_routing_instance() this is the only allocation so far, but
    // callers building multiple instances in one process (e.g. benchmark_main's A/B pair) will
    // see later instances' values include earlier instances' retained memory.
    double process_peak_rss_mb = 0.0;
    std::optional<PreprocessStats> ch; // CH ordering_init/witness_calls, when CH was built
};

struct AlgorithmPreprocessStats {
    std::chrono::nanoseconds wall{0};
    std::chrono::nanoseconds cpu{0};
    // Process-wide high-water RSS sampled right after this phase; see DependencyPreprocessStats
    // for the same multi-instance caveat.
    double process_peak_rss_mb = 0.0;
};

struct PreprocessReport {
    DependencyPreprocessStats dependency;
    AlgorithmPreprocessStats algorithm;
    std::optional<std::string> ch_loaded_from;
    std::optional<std::string> ch_saved_to;
    std::optional<std::string> arcflags_loaded_from;
    std::optional<std::string> arcflags_saved_to;
};

// Bundle returned by make_routing_instance(): the constructed, already-preprocessed algorithm
// plus a breakdown of preprocessing cost between shared-dependency (CH/PHAST) and
// algorithm-specific phases.
struct RoutingInstance {
    std::unique_ptr<RoutingAlgorithm> algorithm;
    PreprocessReport stats;
};

struct RoutingPreprocessingContext {
    std::optional<std::string> ch_load_path;
    std::optional<std::string> ch_save_path;
    std::optional<std::string> arcflags_load_path;
    std::optional<std::string> arcflags_save_path;
};

// Validates `name`/`coords`, builds any CH/PHAST dependency the algorithm needs, constructs the
// algorithm (moving the dependency in), and runs its preprocess(). Returns the algorithm ready to
// query, along with a timing/RSS breakdown of the dependency vs. algorithm preprocessing phases.
[[nodiscard]] RoutingInstance make_routing_instance(const std::string &name, const Graph &graph,
                                                    std::span<const NodeCoord> coords = {});
[[nodiscard]] RoutingInstance make_routing_instance(const std::string &name, const Graph &graph,
                                                    std::span<const NodeCoord> coords,
                                                    const RoutingPreprocessingContext &context);

} // namespace transport
