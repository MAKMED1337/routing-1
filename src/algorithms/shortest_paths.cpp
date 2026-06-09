#include "algorithms/shortest_paths.hpp"

#include <algorithm>

namespace transport {

void dijkstra_one_to_all(const Graph &graph, VertexId source, std::vector<Distance> &out) {
    out.resize(graph.vertex_count());
    if (source >= graph.vertex_count()) {
        std::fill(out.begin(), out.end(), kUnreachable);
        return;
    }
    dijkstra_one_to_all([&graph](VertexId v) { return graph.adjacent_edges(v); }, source, out);
}

} // namespace transport
