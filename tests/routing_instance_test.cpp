#include "algorithms/routing_instance.hpp"
#include "graph_fixtures.hpp"
#include "routing_test_utils.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

using transport::InjectedArcFlags;
using transport::InjectedLandmarks;
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
    for (const std::string &algo :
         {std::string("dijkstra"), std::string("bidijkstra"), std::string("ch"), std::string("alt")}) {
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

bool check_alt_landmark_context() {
    const auto [graph, coords] = transport::test::make_grid_graph(3, 3);
    for (const transport::alt::LandmarkStrategy strategy :
         {transport::alt::LandmarkStrategy::Random, transport::alt::LandmarkStrategy::Farthest,
          transport::alt::LandmarkStrategy::Planar}) {
        transport::RoutingPreprocessingContext context;
        context.landmarks = InjectedLandmarks{strategy, 4, 2};
        transport::RoutingInstance instance = transport::make_routing_instance("alt", graph, coords, context);
        if (!check_all_pairs(graph, *instance.algorithm, "alt configured landmarks")) {
            return false;
        }
    }

    transport::RoutingPreprocessingContext zero_count;
    zero_count.landmarks = InjectedLandmarks{std::nullopt, 0, std::nullopt};
    if (!expect_throws([&] { (void)transport::make_routing_instance("alt", graph, coords, zero_count); },
                       "factory: expected ALT zero landmark count to throw")) {
        return false;
    }

    transport::RoutingPreprocessingContext zero_active;
    zero_active.landmarks = InjectedLandmarks{std::nullopt, std::nullopt, 0};
    if (!expect_throws([&] { (void)transport::make_routing_instance("alt", graph, coords, zero_active); },
                       "factory: expected ALT zero active landmarks to throw")) {
        return false;
    }

    transport::RoutingPreprocessingContext too_many_active;
    too_many_active.landmarks = InjectedLandmarks{std::nullopt, 2, 3};
    if (!expect_throws([&] { (void)transport::make_routing_instance("alt", graph, coords, too_many_active); },
                       "factory: expected ALT active landmarks > total to throw")) {
        return false;
    }

    transport::RoutingPreprocessingContext planar_without_coords;
    planar_without_coords.landmarks =
        InjectedLandmarks{transport::alt::LandmarkStrategy::Planar, std::nullopt, std::nullopt};
    if (!expect_throws([&] { (void)transport::make_routing_instance("alt", graph, {}, planar_without_coords); },
                       "factory: expected planar ALT without coordinates to throw")) {
        return false;
    }

    transport::RoutingPreprocessingContext alt_option_on_dijkstra;
    alt_option_on_dijkstra.landmarks = InjectedLandmarks{std::nullopt, 4, std::nullopt};
    return expect_throws(
        [&] { (void)transport::make_routing_instance("dijkstra", graph, coords, alt_option_on_dijkstra); },
        "factory: expected ALT options on Dijkstra to throw");
}

bool check_dependency_preprocessing_runs_for_ch_dependent_algorithms() {
    const auto [graph, coords] = transport::test::make_grid_graph(3, 3);

    for (const std::string &algo : {std::string("arcflags"), std::string("chase"), std::string("hl")}) {
        transport::RoutingInstance instance = transport::make_routing_instance(algo, graph, coords);
        if (instance.stats.dependency.wall < std::chrono::nanoseconds::zero()) {
            std::cerr << "instance: expected non-negative dependency wall time for '" << algo << "'\n";
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

bool check_ch_artifact_context_round_trip() {
    const transport::Graph graph = transport::test::make_line_graph();
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "transport_context_ch.ch";
    std::filesystem::remove(path);

    transport::RoutingPreprocessingContext save_context;
    save_context.ch_save_path = path.string();
    transport::RoutingInstance saved = transport::make_routing_instance("ch", graph, {}, save_context);
    if (!std::filesystem::exists(path)) {
        std::cerr << "instance: expected CH artifact to be saved\n";
        return false;
    }

    transport::RoutingPreprocessingContext load_context;
    load_context.ch_load_path = path.string();
    transport::RoutingInstance loaded = transport::make_routing_instance("ch", graph, {}, load_context);
    std::filesystem::remove(path);

    return check_all_pairs(graph, *saved.algorithm, "saved ch") &&
           check_all_pairs(graph, *loaded.algorithm, "loaded ch");
}

bool check_arcflags_artifact_context_round_trip() {
    const auto [graph, coords] = transport::test::make_grid_graph(3, 3);
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "transport_context_arcflags.af";
    std::filesystem::remove(path);

    transport::RoutingPreprocessingContext save_context;
    save_context.arcflags = InjectedArcFlags{std::nullopt, path.string(), std::nullopt, std::nullopt, std::nullopt};
    transport::RoutingInstance saved = transport::make_routing_instance("arcflags", graph, coords, save_context);
    if (!std::filesystem::exists(path)) {
        std::cerr << "instance: expected ArcFlags artifact to be saved\n";
        return false;
    }

    transport::RoutingPreprocessingContext load_context;
    load_context.arcflags = InjectedArcFlags{path.string(), std::nullopt, std::nullopt, std::nullopt, std::nullopt};
    transport::RoutingInstance loaded = transport::make_routing_instance("arcflags", graph, {}, load_context);
    std::filesystem::remove(path);

    return check_all_pairs(graph, *saved.algorithm, "saved arcflags") &&
           check_all_pairs(graph, *loaded.algorithm, "loaded arcflags");
}

} // namespace

int main() {
    bool ok = true;
    ok &= check_partition_dependent_algorithms_require_coords();
    ok &= check_partition_dependent_algorithms_work_with_coords();
    ok &= check_graph_only_algorithms_need_no_coords();
    ok &= check_unsupported_algorithm_throws();
    ok &= check_alt_landmark_context();
    ok &= check_dependency_preprocessing_runs_for_ch_dependent_algorithms();
    ok &= check_ch_artifact_context_round_trip();
    ok &= check_arcflags_artifact_context_round_trip();
    if (!ok) {
        std::cerr << "routing instance tests FAILED\n";
        return 1;
    }
    return 0;
}
