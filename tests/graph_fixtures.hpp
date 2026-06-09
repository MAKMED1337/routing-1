#pragma once

#include "graph/graph.hpp"
#include "routing_test_utils.hpp"

namespace transport::test {

// Directed graph where d(a,b) ≠ d(b,a) for many pairs.
// 0 --1--> 1 --1--> 2
//                   |
//                   +--10--> 1  (2→1 costs 10; so d(0,1)=1 but d(2,1)=10)
// Vertex 3 is isolated.
inline Graph make_directed_asymmetric_graph() {
    return make_graph(4, {
                             /*0*/ {Edge{.to = 1, .weight = 1}},
                             /*1*/ {Edge{.to = 2, .weight = 1}},
                             /*2*/ {Edge{.to = 1, .weight = 10}},
                             /*3*/ {},
                         });
}

// Undirected ring: 0 -- 2 -- 1 -- 3 -- 2 -- 1 -- 0 (symmetric weights).
inline Graph make_symmetric_graph() {
    return make_graph(4, {
                             /*0*/ {Edge{.to = 1, .weight = 2}, Edge{.to = 3, .weight = 5}},
                             /*1*/ {Edge{.to = 0, .weight = 2}, Edge{.to = 2, .weight = 3}},
                             /*2*/ {Edge{.to = 1, .weight = 3}, Edge{.to = 3, .weight = 1}},
                             /*3*/ {Edge{.to = 2, .weight = 1}, Edge{.to = 0, .weight = 5}},
                         });
}

} // namespace transport::test
