#include "algorithms/routing_algorithm_factory.hpp"
#include "graph_fixtures.hpp"
#include "routing_test_utils.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {

using transport::test::check_all_pairs;

bool check_astar_requires_coords() {
    const transport::Graph graph = transport::test::make_graph(3, {
                                                                      /*0*/ {{1, 1}},
                                                                      /*1*/ {{2, 1}},
                                                                      /*2*/ {},
                                                                  });

    for (const std::string &algo : {std::string("astar"), std::string("bidi_astar")}) {
        bool threw = false;
        try {
            (void)transport::make_routing_algorithm(algo, graph);
        } catch (const std::invalid_argument &) {
            threw = true;
        }
        if (!threw) {
            std::cerr << "factory: expected '" << algo << "' to throw without coordinates\n";
            return false;
        }

        const std::vector<transport::NodeCoord> wrong_size(graph.vertex_count() - 1);
        threw = false;
        try {
            (void)transport::make_routing_algorithm(algo, graph, wrong_size);
        } catch (const std::invalid_argument &) {
            threw = true;
        }
        if (!threw) {
            std::cerr << "factory: expected '" << algo << "' to throw with mismatched coordinates\n";
            return false;
        }
    }
    return true;
}

bool check_astar_works_with_coords() {
    const transport::test::GraphWithCoords fixture = transport::test::make_grid_graph(3, 3);

    for (const std::string &algo : {std::string("astar"), std::string("bidi_astar")}) {
        auto algorithm = transport::make_routing_algorithm(algo, fixture.graph, fixture.coords);
        algorithm->preprocess();
        if (!check_all_pairs(fixture.graph, *algorithm, algo)) {
            return false;
        }
    }
    return true;
}

bool check_graph_only_algorithms_need_no_coords() {
    const transport::Graph graph = transport::test::make_line_graph();
    for (const std::string &algo : {std::string("dijkstra"), std::string("bidijkstra"), std::string("ch")}) {
        auto algorithm = transport::make_routing_algorithm(algo, graph);
        algorithm->preprocess();
        if (!check_all_pairs(graph, *algorithm, algo)) {
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    bool ok = true;
    ok &= check_astar_requires_coords();
    ok &= check_astar_works_with_coords();
    ok &= check_graph_only_algorithms_need_no_coords();
    if (!ok) {
        std::cerr << "routing algorithm factory tests FAILED\n";
        return 1;
    }
    return 0;
}
