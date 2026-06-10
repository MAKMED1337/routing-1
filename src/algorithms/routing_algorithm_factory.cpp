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
#include <span>
#include <stdexcept>
#include <string>

namespace transport {

std::unique_ptr<RoutingAlgorithm> make_routing_algorithm(const std::string &name, const Graph &graph,
                                                         std::span<const NodeCoord> coords) {
    if (name == "dijkstra") {
        return std::make_unique<DijkstraAlgorithm>(graph);
    }
    if (name == "astar" || name == "bidi_astar" || name == "arcflags" || name == "chase") {
        require_matching_coords(coords, graph.vertex_count(), "algorithm '" + name + "'");
    }
    auto haversine_heuristic = [coords](VertexId from, VertexId to) -> Distance {
        return static_cast<Distance>(
            std::floor(haversine_meters(coords[from], coords[to]) * static_cast<double>(kDistanceScale)));
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
        return std::make_unique<ArcFlagsAlgorithm>(graph, uint16_t{32}, PartitionMethod::Inertial, uint32_t{1}, coords);
    }
    if (name == "chase") {
        return std::make_unique<ChaseAlgorithm>(graph, 0.05, uint16_t{64}, PartitionMethod::Inertial, coords);
    }
    if (name == "hl") {
        return std::make_unique<HubLabelsAlgorithm>(graph);
    }
    throw std::invalid_argument("unsupported algorithm: " + name);
}

} // namespace transport
