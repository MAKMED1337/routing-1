#include "algorithms/routing_algorithm_factory.hpp"
#include "graph_fixtures.hpp"
#include "routing_test_utils.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {

using transport::test::check_all_pairs;
using transport::test::expect_throws;

bool check_partition_dependent_algorithms_require_coords() {
    const transport::Graph graph = transport::test::make_short_chain_graph();

    for (const std::string &algo :
         {std::string("astar"), std::string("bidi_astar"), std::string("arcflags"), std::string("chase")}) {
        if (!expect_throws([&] { (void)transport::make_routing_algorithm(algo, graph); },
                           "factory: expected '" + algo + "' to throw without coordinates")) {
            return false;
        }

        const std::vector<transport::NodeCoord> wrong_size(graph.vertex_count() - 1);
        if (!expect_throws([&] { (void)transport::make_routing_algorithm(algo, graph, wrong_size); },
                           "factory: expected '" + algo + "' to throw with mismatched coordinates")) {
            return false;
        }
    }
    return true;
}

bool check_partition_dependent_algorithms_work_with_coords() {
    const auto [graph, coords] = transport::test::make_grid_graph(3, 3);

    for (const std::string &algo :
         {std::string("astar"), std::string("bidi_astar"), std::string("arcflags"), std::string("chase")}) {
        auto algorithm = transport::make_routing_algorithm(algo, graph, coords);
        algorithm->preprocess();
        if (!check_all_pairs(graph, *algorithm, algo)) {
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
    ok &= check_partition_dependent_algorithms_require_coords();
    ok &= check_partition_dependent_algorithms_work_with_coords();
    ok &= check_graph_only_algorithms_need_no_coords();
    if (!ok) {
        std::cerr << "routing algorithm factory tests FAILED\n";
        return 1;
    }
    return 0;
}
