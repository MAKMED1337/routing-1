#include "algorithms/alt/alt.hpp"
#include "algorithms/alt/landmarks.hpp"
#include "graph/graph.hpp"
#include "graph/reverse_graph.hpp"
#include "routing_test_utils.hpp"

#include <iostream>
#include <random>
#include <vector>

namespace {

using transport::test::check_all_pairs;
using transport::test::make_graph;

bool check_alt_landmark_edges() {
    const transport::Graph disconnected = make_graph(4, {
                                                            {{1, 7}},
                                                            {},
                                                            {{3, 11}},
                                                            {},
                                                        });

    const transport::Graph reverse = transport::build_reverse_graph(disconnected);
    std::mt19937 rng{1};

    transport::alt::LandmarkSet random_landmarks =
        transport::alt::build_landmarks(disconnected, reverse, 10, transport::alt::LandmarkStrategy::Random, rng);
    transport::AltAlgorithm random_alt(disconnected, std::move(random_landmarks), 10);
    if (random_alt.landmark_table_bytes() >
        2ULL * disconnected.vertex_count() * disconnected.vertex_count() * sizeof(transport::Distance)) {
        std::cerr << "ALT stored more landmark table entries than vertices allow\n";
        return false;
    }

    transport::alt::LandmarkSet farthest_landmarks =
        transport::alt::build_landmarks(disconnected, reverse, 10, transport::alt::LandmarkStrategy::Farthest, rng);
    transport::AltAlgorithm farthest_alt(disconnected, std::move(farthest_landmarks), 10);
    if (!check_all_pairs(disconnected, farthest_alt, "alt farthest oversized landmarks")) {
        return false;
    }

    const std::vector<transport::NodeCoord> coords = {
        {50.0, 20.0},
        {50.1, 20.0},
        {51.0, 21.0},
        {51.1, 21.0},
    };
    transport::alt::LandmarkSet planar_landmarks = transport::alt::build_landmarks(
        disconnected, reverse, 10, transport::alt::LandmarkStrategy::Planar, rng, coords);
    transport::AltAlgorithm planar_alt(disconnected, std::move(planar_landmarks), 10);
    return check_all_pairs(disconnected, planar_alt, "alt planar oversized landmarks");
}

} // namespace

int main() {
    if (!check_alt_landmark_edges()) {
        return 1;
    }
    return 0;
}
