#include "algorithms/alt/landmarks.hpp"

#include "algorithms/heap_node.hpp"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <queue>
#include <random>
#include <string>
#include <vector>

namespace transport::alt {
namespace {

std::vector<VertexId> select_random(const Graph &graph, uint32_t landmark_count, std::mt19937 &rng) {
    const VertexId vertices = graph.vertex_count();
    std::vector<VertexId> candidates;
    candidates.reserve(vertices);
    for (VertexId vertex = 0; vertex < vertices; ++vertex) {
        if (graph.offsets[static_cast<size_t>(vertex) + 1] > graph.offsets[vertex]) {
            candidates.push_back(vertex);
        }
    }

    std::shuffle(candidates.begin(), candidates.end(), rng);
    landmark_count = std::min(landmark_count, static_cast<uint32_t>(candidates.size()));
    return std::vector<VertexId>(candidates.begin(), candidates.begin() + landmark_count);
}

std::vector<VertexId> select_farthest(const Graph &graph, uint32_t landmark_count, std::mt19937 &rng) {
    const VertexId vertices = graph.vertex_count();
    if (vertices == 0 || landmark_count == 0) {
        return {};
    }

    landmark_count = std::min(landmark_count, vertices);
    std::uniform_int_distribution<VertexId> pick(0, vertices - 1);

    std::vector<VertexId> chosen;
    chosen.reserve(landmark_count);
    std::vector<Distance> min_dist(vertices, kLandmarkInf);
    std::vector<Distance> tmp(vertices);

    const VertexId seed = pick(rng);
    one_to_all(graph, seed, tmp);

    Distance best_distance = 0;
    VertexId best_vertex = seed;
    for (VertexId vertex = 0; vertex < vertices; ++vertex) {
        if (tmp[vertex] != kLandmarkInf && tmp[vertex] > best_distance) {
            best_distance = tmp[vertex];
            best_vertex = vertex;
        }
    }

    chosen.push_back(best_vertex);
    one_to_all(graph, best_vertex, tmp);
    for (VertexId vertex = 0; vertex < vertices; ++vertex) {
        min_dist[vertex] = std::min(min_dist[vertex], tmp[vertex]);
    }

    for (uint32_t i = 1; i < landmark_count; ++i) {
        best_distance = 0;
        best_vertex = chosen.front();
        for (VertexId vertex = 0; vertex < vertices; ++vertex) {
            if (min_dist[vertex] != kLandmarkInf && min_dist[vertex] > best_distance) {
                best_distance = min_dist[vertex];
                best_vertex = vertex;
            }
        }

        if (std::find(chosen.begin(), chosen.end(), best_vertex) != chosen.end()) {
            for (VertexId vertex = 0; vertex < vertices; ++vertex) {
                if (std::find(chosen.begin(), chosen.end(), vertex) == chosen.end()) {
                    best_vertex = vertex;
                    break;
                }
            }
        }

        chosen.push_back(best_vertex);
        one_to_all(graph, best_vertex, tmp);
        for (VertexId vertex = 0; vertex < vertices; ++vertex) {
            min_dist[vertex] = std::min(min_dist[vertex], tmp[vertex]);
        }
    }

    return chosen;
}

} // namespace

void one_to_all(const Graph &graph, VertexId source, std::vector<Distance> &out) {
    std::fill(out.begin(), out.end(), kLandmarkInf);
    if (source >= graph.vertex_count()) {
        return;
    }

    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
    out[source] = 0;
    pq.push({0, source});
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != out[top.v]) {
            continue;
        }

        for (const Edge &edge : graph.adjacent_edges(top.v)) {
            const Distance next_distance = top.key + edge.weight;
            if (next_distance < out[edge.to]) {
                out[edge.to] = next_distance;
                pq.push({next_distance, edge.to});
            }
        }
    }
}

LandmarkSet build_landmarks(const Graph &graph, const Graph &reverse, uint32_t landmark_count,
                            LandmarkStrategy strategy, uint32_t seed) {
    std::mt19937 rng(seed);
    return build_landmarks(graph, reverse, landmark_count, strategy, rng);
}

LandmarkSet build_landmarks(const Graph &graph, const Graph &reverse, uint32_t landmark_count,
                            LandmarkStrategy strategy, std::mt19937 &rng) {
    const VertexId vertices = graph.vertex_count();
    landmark_count = std::min(landmark_count, vertices);

    std::vector<VertexId> chosen;
    std::string strategy_name;
    switch (strategy) {
    case LandmarkStrategy::Random:
        chosen = select_random(graph, landmark_count, rng);
        strategy_name = "random";
        break;
    case LandmarkStrategy::Farthest:
        chosen = select_farthest(graph, landmark_count, rng);
        strategy_name = "farthest";
        break;
    }

    LandmarkSet landmarks;
    landmarks.landmarks = std::move(chosen);
    landmarks.vertex_count = vertices;
    landmarks.strategy_name = std::move(strategy_name);
    landmarks.dist_from.resize(landmarks.landmarks.size() * static_cast<size_t>(vertices), kLandmarkInf);
    landmarks.dist_to.resize(landmarks.landmarks.size() * static_cast<size_t>(vertices), kLandmarkInf);

    std::vector<Distance> tmp(vertices);
    for (size_t i = 0; i < landmarks.landmarks.size(); ++i) {
        const size_t offset = i * static_cast<size_t>(vertices);
        one_to_all(graph, landmarks.landmarks[i], tmp);
        std::copy(tmp.begin(), tmp.end(), landmarks.dist_from.begin() + static_cast<std::ptrdiff_t>(offset));

        one_to_all(reverse, landmarks.landmarks[i], tmp);
        std::copy(tmp.begin(), tmp.end(), landmarks.dist_to.begin() + static_cast<std::ptrdiff_t>(offset));
    }

    return landmarks;
}

} // namespace transport::alt
