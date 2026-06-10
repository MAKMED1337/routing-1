#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/chase/chase.hpp"
#include "graph_fixtures.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

bool check_chase_validation() {
    const transport::Graph graph = transport::test::make_line_graph();
    transport::ContractionHierarchyAlgorithm ch(graph);
    ch.preprocess();
    bool ok = true;
    for (const double core_fraction : {0.0, -0.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
        try {
            transport::ChaseAlgorithm chase(graph, ch.get_ch(), core_fraction, 4, transport::PartitionMethod::Grid, {});
            chase.preprocess();
            std::cerr << "chase: expected std::invalid_argument for core_fraction=" << core_fraction << "\n";
            ok = false;
        } catch (const std::invalid_argument &) {
        }
    }
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok &= check_chase_validation();
    if (!ok) {
        std::cerr << "chase tests FAILED\n";
        return 1;
    }
    return 0;
}
