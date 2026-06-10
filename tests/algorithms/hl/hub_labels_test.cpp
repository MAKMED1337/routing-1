#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/hl/hub_labels.hpp"
#include "graph_fixtures.hpp"

#include <iostream>
#include <stdexcept>

namespace {

bool check_hl_validation() {
    const transport::Graph graph = transport::test::make_line_graph();
    transport::ContractionHierarchyAlgorithm ch(graph);
    ch.preprocess();
    bool ok = true;
    for (double frac : {0.0, -0.1, 1.1}) {
        try {
            transport::HubLabelsAlgorithm hl(graph, ch.get_ch(), frac, 4ULL * 1024 * 1024 * 1024);
            hl.preprocess();
            std::cerr << "hl: expected std::invalid_argument for label_fraction=" << frac << "\n";
            ok = false;
        } catch (const std::invalid_argument &) {
        }
    }
    return ok;
}

bool check_hl_query_before_preprocess_throws() {
    const transport::Graph graph = transport::test::make_line_graph();
    transport::ContractionHierarchyAlgorithm ch(graph);
    ch.preprocess();
    transport::HubLabelsAlgorithm hl(graph, ch.get_ch(), 1.0, 4ULL * 1024 * 1024 * 1024);
    try {
        (void)hl.query(0, 1);
    } catch (const std::logic_error &) {
        return true;
    }
    std::cerr << "hl: expected std::logic_error when querying before preprocess()\n";
    return false;
}

} // namespace

int main() {
    bool ok = true;
    ok &= check_hl_validation();
    ok &= check_hl_query_before_preprocess_throws();
    if (!ok) {
        std::cerr << "hub_labels tests FAILED\n";
        return 1;
    }
    return 0;
}
