#include "graph/graph.hpp"
#include "graph/reverse_graph.hpp"
#include "routing_test_utils.hpp"

#include <iostream>

namespace {

bool check_reverse_graph() {
    const transport::Graph graph = transport::test::make_graph(3, {
                                                                      /*0*/ {{1, 4}},
                                                                      /*1*/ {{2, 7}},
                                                                      /*2*/ {},
                                                                  });

    const transport::Graph reverse = transport::build_reverse_graph(graph);
    if (reverse.vertex_count() != graph.vertex_count()) {
        std::cerr << "reverse graph: vertex count mismatch\n";
        return false;
    }
    if (reverse.edge_count() != graph.edge_count()) {
        std::cerr << "reverse graph: edge count mismatch\n";
        return false;
    }

    bool found_1_to_0 = false;
    for (const transport::Edge &edge : reverse.adjacent_edges(1)) {
        if (edge.to == 0 && edge.weight == 4) {
            found_1_to_0 = true;
        }
    }
    bool found_2_to_1 = false;
    for (const transport::Edge &edge : reverse.adjacent_edges(2)) {
        if (edge.to == 1 && edge.weight == 7) {
            found_2_to_1 = true;
        }
    }
    if (!found_1_to_0 || !found_2_to_1) {
        std::cerr << "reverse graph: edges not reversed correctly\n";
        return false;
    }
    return true;
}

} // namespace

int main() {
    if (!check_reverse_graph()) {
        std::cerr << "reverse graph tests FAILED\n";
        return 1;
    }
    return 0;
}
