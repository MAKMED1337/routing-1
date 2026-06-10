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

std::unique_ptr<RoutingAlgorithm> build_algorithm(const std::string &name, const Graph &graph,
                                                  std::span<const NodeCoord> coords,
                                                  DependencyPreprocessStats &dependency_stats) {
    if (name == "dijkstra") {
        return std::make_unique<DijkstraAlgorithm>(graph);
    }
    if (name == "astar") {
        return std::make_unique<AStarAlgorithm>(graph, make_haversine_heuristic(coords));
    }
    if (name == "alt") {
        return std::make_unique<AltAlgorithm>(graph);
    }
    if (name == "bidijkstra") {
        return std::make_unique<BidirectionalDijkstraAlgorithm>(graph);
    }
    if (name == "bidi_astar") {
        return std::make_unique<BidirectionalAStarAlgorithm>(graph, make_haversine_heuristic(coords));
    }
    if (name == "ch") {
        return std::make_unique<ContractionHierarchyAlgorithm>(graph);
    }

    Stopwatch dependency_sw;
    if (name == "arcflags") {
        ContractionHierarchyBuildResult built = build_contraction_hierarchy(graph);
        dependency_stats.ch = built.stats;
        PhastAlgorithm phast(built.hierarchy);
        dependency_stats.wall_s = to_seconds(dependency_sw.wall_elapsed());
        dependency_stats.cpu_s = to_seconds(dependency_sw.cpu_elapsed());
        dependency_stats.peak_rss_mb = peak_rss_mb();
        return std::make_unique<ArcFlagsAlgorithm>(graph, std::move(phast), uint16_t{32}, PartitionMethod::Inertial,
                                                   uint32_t{1}, coords);
    }
    if (name == "chase") {
        ContractionHierarchyBuildResult built = build_contraction_hierarchy(graph);
        dependency_stats.ch = built.stats;
        dependency_stats.wall_s = to_seconds(dependency_sw.wall_elapsed());
        dependency_stats.cpu_s = to_seconds(dependency_sw.cpu_elapsed());
        dependency_stats.peak_rss_mb = peak_rss_mb();
        return std::make_unique<ChaseAlgorithm>(graph, std::move(built.hierarchy), 0.05, uint16_t{64},
                                                PartitionMethod::Inertial, coords);
    }
    if (name == "hl") {
        ContractionHierarchyBuildResult built = build_contraction_hierarchy(graph);
        dependency_stats.ch = built.stats;
        dependency_stats.wall_s = to_seconds(dependency_sw.wall_elapsed());
        dependency_stats.cpu_s = to_seconds(dependency_sw.cpu_elapsed());
        dependency_stats.peak_rss_mb = peak_rss_mb();
        return std::make_unique<HubLabelsAlgorithm>(graph, std::move(built.hierarchy));
    }
    throw std::invalid_argument("unsupported algorithm: " + name);
}

} // namespace

RoutingInstance make_routing_instance(const std::string &name, const Graph &graph, std::span<const NodeCoord> coords) {
    if (!known_algorithm_names().contains(name)) {
        throw std::invalid_argument("unsupported algorithm: " + name);
    }
    if (requires_coords(name)) {
        require_matching_coords(coords, graph.vertex_count(), "algorithm '" + name + "'");
    }

    PreprocessReport report;
    std::unique_ptr<RoutingAlgorithm> algorithm = build_algorithm(name, graph, coords, report.dependency);

    Stopwatch algorithm_sw;
    algorithm->preprocess();
    report.algorithm.wall_s = to_seconds(algorithm_sw.wall_elapsed());
    report.algorithm.cpu_s = to_seconds(algorithm_sw.cpu_elapsed());
    report.algorithm.peak_rss_mb = peak_rss_mb();

    return RoutingInstance{std::move(algorithm), report};
}

} // namespace transport
