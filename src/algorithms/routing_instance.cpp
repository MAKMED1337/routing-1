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

} // namespace

RoutingInstance::RoutingInstance(const Graph &graph, std::string algorithm_name, std::span<const NodeCoord> coords)
    : graph_(graph), algorithm_name_(std::move(algorithm_name)), coords_(coords), context_(graph) {}

RoutingAlgorithm &RoutingInstance::algorithm() {
    if (!algorithm_) {
        throw std::logic_error("RoutingInstance::preprocess() must be called before algorithm()");
    }
    return *algorithm_;
}

const RoutingAlgorithm &RoutingInstance::algorithm() const {
    if (!algorithm_) {
        throw std::logic_error("RoutingInstance::preprocess() must be called before algorithm()");
    }
    return *algorithm_;
}

std::unique_ptr<RoutingAlgorithm> RoutingInstance::build_algorithm() {
    if (algorithm_name_ == "dijkstra") {
        return std::make_unique<DijkstraAlgorithm>(graph_);
    }
    if (algorithm_name_ == "astar") {
        return std::make_unique<AStarAlgorithm>(graph_, make_haversine_heuristic(coords_));
    }
    if (algorithm_name_ == "alt") {
        return std::make_unique<AltAlgorithm>(graph_);
    }
    if (algorithm_name_ == "bidijkstra") {
        return std::make_unique<BidirectionalDijkstraAlgorithm>(graph_);
    }
    if (algorithm_name_ == "bidi_astar") {
        return std::make_unique<BidirectionalAStarAlgorithm>(graph_, make_haversine_heuristic(coords_));
    }
    if (algorithm_name_ == "ch") {
        return std::make_unique<ContractionHierarchyAlgorithm>(graph_);
    }
    if (algorithm_name_ == "arcflags") {
        return std::make_unique<ArcFlagsAlgorithm>(graph_, context_.phast(), uint16_t{32}, PartitionMethod::Inertial,
                                                   uint32_t{1}, coords_);
    }
    if (algorithm_name_ == "chase") {
        return std::make_unique<ChaseAlgorithm>(graph_, context_.ch(), 0.05, uint16_t{64}, PartitionMethod::Inertial,
                                                coords_);
    }
    if (algorithm_name_ == "hl") {
        return std::make_unique<HubLabelsAlgorithm>(graph_, context_.ch());
    }
    throw std::invalid_argument("unsupported algorithm: " + algorithm_name_);
}

void RoutingInstance::preprocess() {
    if (preprocessed_) {
        return;
    }

    Stopwatch dependency_sw;
    if (algorithm_name_ == "arcflags") {
        context_.build_phast();
    } else if (algorithm_name_ == "chase" || algorithm_name_ == "hl") {
        context_.build_ch();
    }
    timing_.dependency_wall_s = to_seconds(dependency_sw.wall_elapsed());
    timing_.dependency_cpu_s = to_seconds(dependency_sw.cpu_elapsed());
    timing_.after_dependency_peak_rss_mb = peak_rss_mb();

    algorithm_ = build_algorithm();

    Stopwatch algorithm_sw;
    algorithm_->preprocess();
    timing_.algorithm_wall_s = to_seconds(algorithm_sw.wall_elapsed());
    timing_.algorithm_cpu_s = to_seconds(algorithm_sw.cpu_elapsed());
    timing_.after_algorithm_peak_rss_mb = peak_rss_mb();

    preprocessed_ = true;
}

RoutingInstance make_routing_instance(const std::string &name, const Graph &graph, std::span<const NodeCoord> coords) {
    if (!known_algorithm_names().contains(name)) {
        throw std::invalid_argument("unsupported algorithm: " + name);
    }
    if (requires_coords(name)) {
        require_matching_coords(coords, graph.vertex_count(), "algorithm '" + name + "'");
    }
    return RoutingInstance(graph, name, coords);
}

} // namespace transport
