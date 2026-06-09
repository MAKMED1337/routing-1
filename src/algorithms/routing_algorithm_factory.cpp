#include "algorithms/routing_algorithm_factory.hpp"

#include "algorithms/alt/alt.hpp"
#include "algorithms/arcflags/arc_flags.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/bidirectional_astar.hpp"
#include "algorithms/bidirectional_dijkstra.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/chase/chase.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/hl/hub_labels.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace transport {

std::unique_ptr<RoutingAlgorithm> make_routing_algorithm(const std::string &name, const Graph &graph) {
    if (name == "dijkstra") {
        return std::make_unique<DijkstraAlgorithm>(graph);
    }
    auto haversine_heuristic = [&graph](VertexId from, VertexId to) -> Distance {
        return static_cast<Distance>(
            std::floor(haversine_meters(graph.coords[from], graph.coords[to]) * static_cast<double>(kDistanceScale)));
    };
    if (name == "astar") {
        return std::make_unique<AStarAlgorithm>(graph, haversine_heuristic);
    }
    if (name == "alt") {
        return std::make_unique<AltAlgorithm>(graph);
    }
    if (name == "bidijkstra") {
        return std::make_unique<BidirectionalDijkstraAlgorithm>(graph);
    }
    if (name == "bidi_astar") {
        return std::make_unique<BidirectionalAStarAlgorithm>(graph, haversine_heuristic);
    }
    if (name == "ch") {
        return std::make_unique<ContractionHierarchyAlgorithm>(graph);
    }
    if (name == "arcflags") {
        return std::make_unique<ArcFlagsAlgorithm>(graph);
    }
    if (name == "chase") {
        return std::make_unique<ChaseAlgorithm>(graph);
    }
    if (name == "hl") {
        return std::make_unique<HubLabelsAlgorithm>(graph);
    }
    throw std::invalid_argument("unsupported algorithm: " + name);
}

} // namespace transport
