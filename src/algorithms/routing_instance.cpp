#include "algorithms/routing_instance.hpp"

#include "algorithms/alt/alt.hpp"
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
#include "io/arc_flags_io.hpp"
#include "io/ch_io.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

namespace transport {

namespace {

Heuristic make_haversine_heuristic(std::span<const NodeCoord> coords) {
    return [coords](VertexId from, VertexId to) -> Distance {
        return static_cast<Distance>(
            std::floor(haversine_meters(coords[from], coords[to]) * static_cast<double>(kDistanceScale)));
    };
}

const std::set<std::string> &known_algorithm_names() {
    static const std::set<std::string> names = {"dijkstra", "astar",    "alt",   "bidijkstra", "bidi_astar",
                                                "ch",       "arcflags", "chase", "hl"};
    return names;
}

bool requires_coords(const std::string &name) {
    return name == "astar" || name == "bidi_astar" || name == "arcflags" || name == "chase";
}

bool accepts_ch_artifact(const std::string &name) {
    return name == "ch" || name == "arcflags" || name == "chase" || name == "hl";
}

void validate_context(const std::string &name, const RoutingPreprocessingContext &context) {
    if ((context.ch_load_path || context.ch_save_path) && !accepts_ch_artifact(name)) {
        throw std::invalid_argument("CH artifact options are unsupported for algorithm '" + name + "'");
    }
    if ((context.arcflags_load_path || context.arcflags_save_path) && name != "arcflags") {
        throw std::invalid_argument("ArcFlags artifact options are unsupported for algorithm '" + name + "'");
    }
    if (context.arcflags_load_path && context.arcflags_save_path) {
        throw std::invalid_argument("cannot use both arcflags load and save paths");
    }
    if ((context.hl_label_fraction || context.hl_memory_budget_gb) && name != "hl") {
        throw std::invalid_argument("HL options are unsupported for algorithm '" + name + "'");
    }
    if (context.hl_label_fraction && (*context.hl_label_fraction <= 0.0 || *context.hl_label_fraction > 1.0)) {
        throw std::invalid_argument("hl_label_fraction must be in (0, 1]");
    }
    if (context.hl_memory_budget_gb && *context.hl_memory_budget_gb <= 0.0) {
        throw std::invalid_argument("hl_memory_budget_gb must be positive");
    }
    if ((context.arcflags_regions || context.arcflags_partition || context.arcflags_threads) && name != "arcflags") {
        throw std::invalid_argument("Arc Flags options are unsupported for algorithm '" + name + "'");
    }
    if (context.arcflags_threads && *context.arcflags_threads == 0) {
        throw std::invalid_argument("arcflags_threads must be >= 1");
    }
    if ((context.alt_landmark_strategy || context.alt_landmark_count || context.alt_active_landmarks) &&
        name != "alt") {
        throw std::invalid_argument("ALT options are unsupported for algorithm '" + name + "'");
    }
    if (context.alt_landmark_count && *context.alt_landmark_count == 0) {
        throw std::invalid_argument("alt_landmark_count must be >= 1");
    }
    if (context.alt_active_landmarks && *context.alt_active_landmarks == 0) {
        throw std::invalid_argument("alt_active_landmarks must be >= 1");
    }
    const uint32_t landmark_count = context.alt_landmark_count.value_or(uint32_t{16});
    const uint32_t active_landmarks = context.alt_active_landmarks.value_or(uint32_t{4});
    if (active_landmarks > landmark_count) {
        throw std::invalid_argument("alt_active_landmarks must be <= alt_landmark_count");
    }
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
                                                  PreprocessReport &report) {
    if (name == "dijkstra") {
        return std::make_unique<DijkstraAlgorithm>(graph);
    }
    if (name == "astar") {
        return std::make_unique<AStarAlgorithm>(graph, make_haversine_heuristic(coords));
    }
    if (name == "alt") {
        const alt::LandmarkStrategy strategy = context.alt_landmark_strategy.value_or(alt::LandmarkStrategy::Farthest);
        const uint32_t landmark_count = context.alt_landmark_count.value_or(uint32_t{16});
        const uint32_t active_landmarks = context.alt_active_landmarks.value_or(uint32_t{4});
        return std::make_unique<AltAlgorithm>(graph, strategy, landmark_count, active_landmarks, coords);
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
        if (context.arcflags_load_path) {
            ArcFlagsPreprocessedData data = arcflags::load_arc_flags(*context.arcflags_load_path);
            report.arcflags_loaded_from = context.arcflags_load_path->string();
            return std::make_unique<ArcFlagsAlgorithm>(graph, std::move(data));
        }
        const uint16_t regions = context.arcflags_regions.value_or(uint16_t{32});
        const PartitionMethod partition = context.arcflags_partition.value_or(PartitionMethod::Inertial);
        const uint32_t threads = context.arcflags_threads.value_or(uint32_t{1});
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
        const uint64_t memory_budget =
            context.hl_memory_budget_gb ? static_cast<uint64_t>(*context.hl_memory_budget_gb * 1024.0 * 1024.0 * 1024.0)
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
    if (!known_algorithm_names().contains(name)) {
        throw std::invalid_argument("unsupported algorithm: " + name);
    }
    validate_context(name, context);
    const bool coords_are_satisfied_by_artifact = name == "arcflags" && context.arcflags_load_path.has_value();
    const bool alt_planar_requires_coords =
        name == "alt" && context.alt_landmark_strategy == alt::LandmarkStrategy::Planar;
    if ((requires_coords(name) || alt_planar_requires_coords) && !coords_are_satisfied_by_artifact) {
        require_matching_coords(coords, graph.vertex_count(), "algorithm '" + name + "'");
    }

    PreprocessReport report;
    std::unique_ptr<RoutingAlgorithm> algorithm =
        build_algorithm(name, graph, coords, context, report.dependency, report);

    Stopwatch algorithm_sw;
    algorithm->preprocess();
    report.algorithm.wall = algorithm_sw.wall_elapsed();
    report.algorithm.cpu = algorithm_sw.cpu_elapsed();
    report.algorithm.process_peak_rss_mb = peak_rss_mb();

    if (context.ch_save_path && name == "ch") {
        const auto *ch_algorithm = dynamic_cast<const ContractionHierarchyAlgorithm *>(algorithm.get());
        if (ch_algorithm == nullptr || !ch::save_ch(ch_algorithm->get_ch(), *context.ch_save_path)) {
            throw std::runtime_error("failed to save CH artifact: " + context.ch_save_path->string());
        }
        report.ch_saved_to = context.ch_save_path->string();
    }
    if (context.arcflags_save_path) {
        const auto *arcflags_algorithm = dynamic_cast<const ArcFlagsAlgorithm *>(algorithm.get());
        if (arcflags_algorithm == nullptr ||
            !arcflags::save_arc_flags(arcflags_algorithm->export_preprocessed(), *context.arcflags_save_path)) {
            throw std::runtime_error("failed to save ArcFlags artifact: " + context.arcflags_save_path->string());
        }
        report.arcflags_saved_to = context.arcflags_save_path->string();
    }

    return RoutingInstance{std::move(algorithm), report};
}

} // namespace transport
