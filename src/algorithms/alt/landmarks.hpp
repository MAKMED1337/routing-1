#pragma once

#include "graph/graph.hpp"
#include "graph/types.hpp"
#include "routing/routing.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace transport::alt {

constexpr Distance kLandmarkInf = kUnreachable;

enum class LandmarkStrategy { Random, Farthest };

struct LandmarkSet {
    std::vector<VertexId> landmarks;
    std::vector<Distance> dist_from;
    std::vector<Distance> dist_to;
    uint32_t vertex_count = 0;
    std::string strategy_name;
};

void one_to_all(const Graph &graph, VertexId source, std::vector<Distance> &out);

LandmarkSet build_landmarks(const Graph &graph, const Graph &reverse, uint32_t landmark_count,
                            LandmarkStrategy strategy, uint32_t seed);
LandmarkSet build_landmarks(const Graph &graph, const Graph &reverse, uint32_t landmark_count,
                            LandmarkStrategy strategy, std::mt19937 &rng);

} // namespace transport::alt
