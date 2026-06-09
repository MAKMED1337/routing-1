#include "algorithms/shortest_paths.hpp"

#include <stdexcept>

namespace transport {

void dijkstra_one_to_all(const Graph &graph, VertexId source, std::vector<Distance> &out) {
    if (source >= graph.vertex_count()) {
        throw std::out_of_range("source vertex out of range");
    }
    out.resize(graph.vertex_count());
    dijkstra_one_to_all([&graph](VertexId v) { return graph.adjacent_edges(v); }, source, out);
}

} // namespace transport
