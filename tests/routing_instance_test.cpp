#include "algorithms/routing_instance.hpp"
#include "graph_fixtures.hpp"
#include "routing_test_utils.hpp"

#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

using transport::test::check_all_pairs;
using transport::test::expect_throws;

bool check_partition_dependent_algorithms_require_coords() {
    const transport::Graph graph = transport::test::make_short_chain_graph();

    for (const std::string &algo :
         {std::string("astar"), std::string("bidi_astar"), std::string("arcflags"), std::string("chase")}) {
        if (!expect_throws([&] { (void)transport::make_routing_instance(algo, graph); },
                           "factory: expected '" + algo + "' to throw without coordinates")) {
            return false;
        }

        const std::vector<transport::NodeCoord> wrong_size(graph.vertex_count() - 1);
        if (!expect_throws([&] { (void)transport::make_routing_instance(algo, graph, wrong_size); },
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
        transport::RoutingInstance instance = transport::make_routing_instance(algo, graph, coords);
        if (!check_all_pairs(graph, *instance.algorithm, algo)) {
            return false;
        }
    }
    return true;
}

bool check_graph_only_algorithms_need_no_coords() {
    const transport::Graph graph = transport::test::make_line_graph();
    for (const std::string &algo : {std::string("dijkstra"), std::string("bidijkstra"), std::string("ch")}) {
        transport::RoutingInstance instance = transport::make_routing_instance(algo, graph);
        if (!check_all_pairs(graph, *instance.algorithm, algo)) {
            return false;
        }
    }
    return true;
}

bool check_unsupported_algorithm_throws() {
    const transport::Graph graph = transport::test::make_line_graph();
    return expect_throws([&] { (void)transport::make_routing_instance("not-an-algorithm", graph); },
                         "factory: expected unsupported algorithm name to throw");
}

bool check_dependency_preprocessing_runs_for_ch_dependent_algorithms() {
    const auto [graph, coords] = transport::test::make_grid_graph(3, 3);

    for (const std::string &algo : {std::string("arcflags"), std::string("chase"), std::string("hl")}) {
        transport::RoutingInstance instance = transport::make_routing_instance(algo, graph, coords);
        if (instance.stats.dependency.wall_s < 0.0) {
            std::cerr << "instance: expected non-negative dependency wall_s for '" << algo << "'\n";
            return false;
        }
        if (!instance.stats.dependency.ch.has_value()) {
            std::cerr << "instance: expected nested CH stats for '" << algo << "'\n";
            return false;
        }
        if (!check_all_pairs(graph, *instance.algorithm, algo)) {
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
    ok &= check_unsupported_algorithm_throws();
    ok &= check_dependency_preprocessing_runs_for_ch_dependent_algorithms();
    if (!ok) {
        std::cerr << "routing instance tests FAILED\n";
        return 1;
    }
    return 0;
}
