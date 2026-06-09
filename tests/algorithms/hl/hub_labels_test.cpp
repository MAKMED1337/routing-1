#include "algorithms/hl/hub_labels.hpp"
#include "graph_fixtures.hpp"

#include <iostream>
#include <stdexcept>

namespace {

bool check_hl_validation() {
    const transport::Graph graph = transport::test::make_line_graph();
    bool ok = true;
    for (double frac : {0.0, -0.1, 1.1}) {
        try {
            transport::HubLabelsAlgorithm hl(graph, frac, 4ULL * 1024 * 1024 * 1024, 1);
            hl.preprocess();
            std::cerr << "hl: expected std::invalid_argument for label_fraction=" << frac << "\n";
            ok = false;
        } catch (const std::invalid_argument &) {
        }
    }
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok &= check_hl_validation();
    if (!ok) {
        std::cerr << "hub_labels tests FAILED\n";
        return 1;
    }
    return 0;
}
