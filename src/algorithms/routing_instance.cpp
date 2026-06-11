#include "algorithms/routing_instance.hpp"

#include "algorithms/alt/alt.hpp"
#include "algorithms/alt/landmarks.hpp"
#include "algorithms/arcflags/arc_flags.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/bidirectional_astar.hpp"
#include "algorithms/bidirectional_dijkstra.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/chase/chase.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/heuristic.hpp"
#include "algorithms/hl/hub_labels.hpp"
#include "algorithms/partition.hpp"
#include "algorithms/phast.hpp"
#include "algorithms/stopwatch.hpp"
#include "graph/reverse_graph.hpp"
#include "io/arc_flags_io.hpp"
#include "io/ch_io.hpp"

#include <cmath>
#include <map>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

namespace transport {

namespace {

Heuristic make_haversine_heuristic(std::span<const NodeCoord> coords) {
    return [coords](VertexId from, VertexId to) -> Distance {
        return static_cast<Distance>(
            std::floor(haversine_meters(coords[from], coords[to]) * static_cast<double>(kDistanceScale)));
    };
}

struct AlgorithmParameters {
    bool needs_coords = false;
    bool uses_ch_artifact = false;
    bool uses_arcflags = false;
    bool uses_landmarks = false;
    bool uses_hl = false;

    void validate(const std::string &name, const RoutingPreprocessingContext &context) const {
        if ((context.ch_load_path || context.ch_save_path) && !uses_ch_artifact) {
            throw std::invalid_argument("CH artifact options are unsupported for algorithm '" + name + "'");
        }
        if (context.arcflags && !uses_arcflags) {
            throw std::invalid_argument("ArcFlags options are unsupported for algorithm '" + name + "'");
        }
        if (context.arcflags && context.arcflags->load_path && context.arcflags->save_path) {
            throw std::invalid_argument("cannot use both arcflags load and save paths");
        }
        if ((context.hl_label_fraction || context.memory_budget_gb) && !uses_hl) {
            throw std::invalid_argument("HL options are unsupported for algorithm '" + name + "'");
        }
        if (context.landmarks && !uses_landmarks) {
            throw std::invalid_argument("ALT options are unsupported for algorithm '" + name + "'");
        }
        if (context.hl_label_fraction && (*context.hl_label_fraction <= 0.0 || *context.hl_label_fraction > 1.0)) {
            throw std::invalid_argument("hl_label_fraction must be in (0, 1]");
        }
        if (context.memory_budget_gb && *context.memory_budget_gb <= 0.0) {
            throw std::invalid_argument("memory_budget_gb must be positive");
        }
        if (context.arcflags && context.arcflags->threads && *context.arcflags->threads == 0) {
            throw std::invalid_argument("arcflags_threads must be >= 1");
        }
        if (context.landmarks) {
            if (context.landmarks->count && *context.landmarks->count == 0) {
                throw std::invalid_argument("alt_landmark_count must be >= 1");
            }
            if (context.landmarks->active && *context.landmarks->active == 0) {
                throw std::invalid_argument("alt_active_landmarks must be >= 1");
            }
            const uint32_t count = context.landmarks->count.value_or(uint32_t{16});
            const uint32_t active = context.landmarks->active.value_or(uint32_t{4});
            if (active > count) {
                throw std::invalid_argument("alt_active_landmarks must be <= alt_landmark_count");
            }
        }
    }
};

const std::map<std::string, AlgorithmParameters> &algorithm_registry() {
    static const std::map<std::string, AlgorithmParameters> reg = {
        {"dijkstra", {false, false, false, false, false}},  {"astar", {true, false, false, false, false}},
        {"alt", {false, false, false, true, false}},        {"bidijkstra", {false, false, false, false, false}},
        {"bidi_astar", {true, false, false, false, false}}, {"ch", {false, true, false, false, false}},
        {"arcflags", {true, true, true, false, false}},     {"chase", {true, true, false, false, false}},
        {"hl", {false, true, false, false, true}},
    };
    return reg;
}

ContractionHierarchyBuildResult build_or_load_ch(const Graph &graph, const RoutingPreprocessingContext &context,
                                                 DependencyPreprocessStats &dependency_stats,
                                                 PreprocessReport &report) {
    Stopwatch dependency_sw;
    if (context.ch_load_path) {
        ContractionHierarchyBuildResult result;
        result.hierarchy = ch::load_ch(*context.ch_load_path);
        if (result.hierarchy.vertex_count() != graph.vertex_count()) {
            throw std::invalid_argument("loaded CH vertex count does not match graph");
        }
        dependency_stats.wall = dependency_sw.wall_elapsed();
        dependency_stats.cpu = dependency_sw.cpu_elapsed();
        dependency_stats.process_peak_rss_mb = peak_rss_mb();
        report.ch_loaded_from = context.ch_load_path->string();
        return result;
    }

    ContractionHierarchyBuildResult result = build_contraction_hierarchy(graph);
    dependency_stats.ch = result.stats;
    dependency_stats.wall = dependency_sw.wall_elapsed();
    dependency_stats.cpu = dependency_sw.cpu_elapsed();
    dependency_stats.process_peak_rss_mb = peak_rss_mb();
    if (context.ch_save_path) {
        if (!ch::save_ch(result.hierarchy, *context.ch_save_path)) {
            throw std::runtime_error("failed to save CH artifact: " + context.ch_save_path->string());
        }
        report.ch_saved_to = context.ch_save_path->string();
    }
    return result;
}

std::unique_ptr<RoutingAlgorithm> build_algorithm(const std::string &name, const Graph &graph,
                                                  std::span<const NodeCoord> coords,
                                                  const RoutingPreprocessingContext &context,
                                                  DependencyPreprocessStats &dependency_stats,
                                                  AlgorithmPreprocessStats &algorithm_stats, PreprocessReport &report) {
    if (name == "dijkstra") {
        return std::make_unique<DijkstraAlgorithm>(graph);
    }
    if (name == "astar") {
        return std::make_unique<AStarAlgorithm>(graph, make_haversine_heuristic(coords));
    }
    if (name == "alt") {
        const alt::LandmarkStrategy strategy =
            context.landmarks ? context.landmarks->strategy.value_or(alt::LandmarkStrategy::Farthest)
                              : alt::LandmarkStrategy::Farthest;
        const uint32_t count = context.landmarks ? context.landmarks->count.value_or(uint32_t{16}) : uint32_t{16};
        const uint32_t active = context.landmarks ? context.landmarks->active.value_or(uint32_t{4}) : uint32_t{4};
        const Stopwatch alt_sw;
        std::mt19937 rng{42};
        const Graph reverse = build_reverse_graph(graph);
        alt::LandmarkSet landmarks = alt::build_landmarks(graph, reverse, count, strategy, rng, coords);
        algorithm_stats.wall = alt_sw.wall_elapsed();
        algorithm_stats.cpu = alt_sw.cpu_elapsed();
        return std::make_unique<AltAlgorithm>(graph, std::move(landmarks), active);
    }
    if (name == "bidijkstra") {
        return std::make_unique<BidirectionalDijkstraAlgorithm>(graph);
    }
    if (name == "bidi_astar") {
        return std::make_unique<BidirectionalAStarAlgorithm>(graph, make_haversine_heuristic(coords));
    }
    if (name == "ch") {
        if (context.ch_load_path) {
            ContractionHierarchyBuildResult built = build_or_load_ch(graph, context, dependency_stats, report);
            return std::make_unique<ContractionHierarchyAlgorithm>(graph, std::move(built.hierarchy));
        }
        return std::make_unique<ContractionHierarchyAlgorithm>(graph);
    }
    if (name == "arcflags") {
        const InjectedArcFlags &af = context.arcflags.value_or(InjectedArcFlags{});
        if (af.load_path) {
            ArcFlagsPreprocessedData data = arcflags::load_arc_flags(*af.load_path);
            report.arcflags_loaded_from = af.load_path->string();
            return std::make_unique<ArcFlagsAlgorithm>(graph, std::move(data));
        }
        const uint16_t regions = af.regions.value_or(uint16_t{32});
        const PartitionMethod partition = af.partition.value_or(PartitionMethod::Inertial);
        const uint32_t threads = af.threads.value_or(uint32_t{1});
        ContractionHierarchyBuildResult built = build_or_load_ch(graph, context, dependency_stats, report);
        PhastAlgorithm phast(built.hierarchy);
        return std::make_unique<ArcFlagsAlgorithm>(graph, std::move(phast), regions, partition, threads, coords);
    }
    if (name == "chase") {
        ContractionHierarchyBuildResult built = build_or_load_ch(graph, context, dependency_stats, report);
        return std::make_unique<ChaseAlgorithm>(graph, std::move(built.hierarchy), 0.05, uint16_t{64},
                                                PartitionMethod::Inertial, coords);
    }
    if (name == "hl") {
        ContractionHierarchyBuildResult built = build_or_load_ch(graph, context, dependency_stats, report);
        const double label_fraction = context.hl_label_fraction.value_or(0.25);
        const uint64_t memory_budget = context.memory_budget_gb
                                           ? static_cast<uint64_t>(*context.memory_budget_gb * 1024.0 * 1024.0 * 1024.0)
                                           : 18ULL * 1024 * 1024 * 1024;
        return std::make_unique<HubLabelsAlgorithm>(graph, std::move(built.hierarchy), label_fraction, memory_budget);
    }
    throw std::invalid_argument("unsupported algorithm: " + name);
}

} // namespace

RoutingInstance make_routing_instance(const std::string &name, const Graph &graph, std::span<const NodeCoord> coords) {
    return make_routing_instance(name, graph, coords, RoutingPreprocessingContext{});
}

RoutingInstance make_routing_instance(const std::string &name, const Graph &graph, std::span<const NodeCoord> coords,
                                      const RoutingPreprocessingContext &context) {
    const auto &reg = algorithm_registry();
    const auto it = reg.find(name);
    if (it == reg.end()) {
        throw std::invalid_argument("unsupported algorithm: " + name);
    }
    const AlgorithmParameters &params = it->second;
    params.validate(name, context);

    const bool coords_are_satisfied_by_artifact =
        name == "arcflags" && context.arcflags && context.arcflags->load_path.has_value();
    const bool alt_planar_requires_coords =
        name == "alt" && context.landmarks && context.landmarks->strategy == alt::LandmarkStrategy::Planar;
    if ((params.needs_coords || alt_planar_requires_coords) && !coords_are_satisfied_by_artifact) {
        require_matching_coords(coords, graph.vertex_count(), "algorithm '" + name + "'");
    }

    PreprocessReport report;
    std::unique_ptr<RoutingAlgorithm> algorithm =
        build_algorithm(name, graph, coords, context, report.dependency, report.algorithm, report);

    Stopwatch algorithm_sw;
    algorithm->preprocess();
    report.algorithm.wall += algorithm_sw.wall_elapsed();
    report.algorithm.cpu += algorithm_sw.cpu_elapsed();
    report.algorithm.process_peak_rss_mb = peak_rss_mb();

    if (context.ch_save_path && name == "ch") {
        const auto *ch_algorithm = dynamic_cast<const ContractionHierarchyAlgorithm *>(algorithm.get());
        if (ch_algorithm == nullptr || !ch::save_ch(ch_algorithm->get_ch(), *context.ch_save_path)) {
            throw std::runtime_error("failed to save CH artifact: " + context.ch_save_path->string());
        }
        report.ch_saved_to = context.ch_save_path->string();
    }
    if (context.arcflags && context.arcflags->save_path) {
        const auto *arcflags_algorithm = dynamic_cast<const ArcFlagsAlgorithm *>(algorithm.get());
        if (arcflags_algorithm == nullptr ||
            !arcflags::save_arc_flags(arcflags_algorithm->export_preprocessed(), *context.arcflags->save_path)) {
            throw std::runtime_error("failed to save ArcFlags artifact: " + context.arcflags->save_path->string());
        }
        report.arcflags_saved_to = context.arcflags->save_path->string();
    }

    return RoutingInstance{std::move(algorithm), report};
}

} // namespace transport
