#pragma once

#include "graph/types.hpp"

#include <functional>
#include <queue>
#include <vector>

namespace transport {

// Min-heap entry for Dijkstra-family priority queues: ordered by ascending key
// (use with std::greater<> so the smallest key is popped first).
struct HeapNode {
    Distance key = 0;
    VertexId v = 0;
    bool operator>(const HeapNode &other) const { return key > other.key; }
};

using HeapQueue = std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>>;

} // namespace transport
