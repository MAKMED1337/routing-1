#include "algorithms/shortest_paths.hpp"

namespace transport {

void dijkstra_one_to_all(const Graph &graph, VertexId source, std::vector<Distance> &out) {
    out.assign(graph.vertex_count(), kUnreachable);
    if (source >= graph.vertex_count()) {
        return;
    }
    dijkstra_one_to_all([&graph](VertexId v) { return graph.adjacent_edges(v); }, source, out);
}

} // namespace transport
