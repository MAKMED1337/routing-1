#include "algorithms/alt/alt.hpp"
#include "algorithms/alt/landmarks.hpp"
#include "graph/graph.hpp"
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

    transport::AltAlgorithm random_alt(disconnected, 10, transport::alt::LandmarkStrategy::Random, 10, std::mt19937{1});
    random_alt.preprocess();
    if (random_alt.query(0, 0).distance_units != 0) {
        std::cerr << "ALT source==target query did not return zero\n";
        return false;
    }
    if (random_alt.query(0, 3).distance_units != transport::kUnreachable) {
        std::cerr << "ALT disconnected query should be unreachable\n";
        return false;
    }
    if (random_alt.landmark_table_bytes() >
        2ULL * disconnected.vertex_count() * disconnected.vertex_count() * sizeof(transport::Distance)) {
        std::cerr << "ALT stored more landmark table entries than vertices allow\n";
        return false;
    }

    transport::AltAlgorithm farthest_alt(disconnected, 10, transport::alt::LandmarkStrategy::Farthest, 10,
                                         std::mt19937{1});
    farthest_alt.preprocess();
    if (!check_all_pairs(disconnected, farthest_alt, "alt farthest oversized landmarks")) {
        return false;
    }

    transport::AltAlgorithm planar_alt(disconnected, 10, transport::alt::LandmarkStrategy::Planar, 10, std::mt19937{1});
    planar_alt.preprocess();
    return check_all_pairs(disconnected, planar_alt, "alt planar oversized landmarks");
}

} // namespace

int main() {
    if (!check_alt_landmark_edges()) {
        return 1;
    }
    return 0;
}
