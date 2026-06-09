#include "algorithms/alt/landmarks.hpp"

#include "algorithms/heap_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <numbers>
#include <queue>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace transport::alt {
namespace {

void one_to_all(const Graph &graph, VertexId source, std::vector<Distance> &out) {
    std::fill(out.begin(), out.end(), kUnreachable);
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

std::vector<size_t> select_random(const Graph &graph, uint32_t landmark_count, std::mt19937 &rng) {
    const size_t vertices = graph.vertex_count();
    std::vector<size_t> candidates;
    candidates.reserve(vertices);
    for (size_t vertex = 0; vertex < vertices; ++vertex) {
        if (graph.offsets[vertex + 1] > graph.offsets[vertex]) {
            candidates.push_back(vertex);
        }
    }

    std::shuffle(candidates.begin(), candidates.end(), rng);
    landmark_count = std::min(landmark_count, static_cast<uint32_t>(candidates.size()));
    return std::vector<size_t>(candidates.begin(), candidates.begin() + landmark_count);
}

std::vector<size_t> select_farthest(const Graph &graph, uint32_t landmark_count, std::mt19937 &rng) {
    const size_t vertices = graph.vertex_count();
    if (vertices == 0 || landmark_count == 0) {
        return {};
    }

    landmark_count = std::min(landmark_count, static_cast<uint32_t>(vertices));
    std::uniform_int_distribution<size_t> pick(0, vertices - 1);

    std::vector<size_t> chosen;
    chosen.reserve(landmark_count);
    std::vector<bool> selected(vertices, false);
    std::vector<Distance> min_dist(vertices, kUnreachable);
    std::vector<Distance> tmp(vertices);

    const size_t seed = pick(rng);
    one_to_all(graph, static_cast<VertexId>(seed), tmp);

    std::pair<Distance, size_t> best = {0, seed};
    for (size_t vertex = 0; vertex < vertices; ++vertex) {
        if (tmp[vertex] != kUnreachable) {
            best = std::max(best, std::pair<Distance, size_t>{tmp[vertex], vertex});
        }
    }

    chosen.push_back(best.second);
    selected[best.second] = true;
    one_to_all(graph, static_cast<VertexId>(best.second), tmp);
    for (size_t vertex = 0; vertex < vertices; ++vertex) {
        min_dist[vertex] = std::min(min_dist[vertex], tmp[vertex]);
    }

    for (uint32_t i = 1; i < landmark_count; ++i) {
        best = {0, 0};
        for (size_t vertex = 0; vertex < vertices; ++vertex) {
            if (!selected[vertex]) {
                best = std::max(best, std::pair<Distance, size_t>{min_dist[vertex], vertex});
            }
        }

        chosen.push_back(best.second);
        selected[best.second] = true;
        one_to_all(graph, static_cast<VertexId>(best.second), tmp);
        for (size_t vertex = 0; vertex < vertices; ++vertex) {
            min_dist[vertex] = std::min(min_dist[vertex], tmp[vertex]);
        }
    }

    return chosen;
}

std::vector<size_t> select_planar(const Graph &graph, uint32_t landmark_count) {
    const size_t vertices = graph.vertex_count();
    if (vertices == 0 || landmark_count == 0) {
        return {};
    }

    landmark_count = std::min(landmark_count, static_cast<uint32_t>(vertices));

    double sum_lat = 0.0;
    double sum_lon = 0.0;
    size_t coord_count = 0;
    for (const NodeCoord &coord : graph.coords) {
        if (coord.lat != 0.0 || coord.lon != 0.0) {
            sum_lat += coord.lat;
            sum_lon += coord.lon;
            ++coord_count;
        }
    }
    if (coord_count == 0) {
        throw std::invalid_argument("planar ALT landmark strategy requires coordinates");
    }

    const double center_lat = sum_lat / static_cast<double>(coord_count);
    const double center_lon = sum_lon / static_cast<double>(coord_count);

    std::vector<std::pair<double, size_t>> best_by_sector(landmark_count, {-1.0, 0});
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    for (size_t vertex = 0; vertex < vertices; ++vertex) {
        const NodeCoord &coord = graph.coords[vertex];
        const double angle = std::atan2(coord.lat - center_lat, coord.lon - center_lon);
        const double normalized = (angle + std::numbers::pi) / kTwoPi;
        const size_t sector = static_cast<size_t>(normalized * static_cast<double>(landmark_count)) % landmark_count;
        const double dlat = coord.lat - center_lat;
        const double dlon = coord.lon - center_lon;
        const double squared_distance = dlat * dlat + dlon * dlon;
        best_by_sector[sector] = std::max(best_by_sector[sector], std::pair<double, size_t>{squared_distance, vertex});
    }

    std::vector<size_t> chosen;
    chosen.reserve(landmark_count);
    for (const auto &[distance, vertex] : best_by_sector) {
        if (distance >= 0.0) {
            chosen.push_back(vertex);
        }
    }

    for (size_t vertex = 0; chosen.size() < landmark_count && vertex < vertices; ++vertex) {
        if (std::find(chosen.begin(), chosen.end(), vertex) == chosen.end()) {
            chosen.push_back(vertex);
        }
    }

    return chosen;
}

} // namespace

LandmarkSet build_landmarks(const Graph &graph, const Graph &reverse, uint32_t landmark_count,
                            LandmarkStrategy strategy, std::mt19937 &rng) {
    const size_t vertices = graph.vertex_count();
    landmark_count = std::min(landmark_count, static_cast<uint32_t>(vertices));

    std::vector<size_t> chosen;
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
    case LandmarkStrategy::Planar:
        chosen = select_planar(graph, landmark_count);
        strategy_name = "planar";
        break;
    default:
        throw std::invalid_argument("unknown ALT landmark strategy");
    }

    const size_t chosen_count = chosen.size();
    LandmarkSet landmarks{
        .landmarks = std::move(chosen),
        .dist_from = std::vector<Distance>(chosen_count * vertices, kUnreachable),
        .dist_to = std::vector<Distance>(chosen_count * vertices, kUnreachable),
        .vertex_count = vertices,
        .strategy_name = std::move(strategy_name),
    };

    std::vector<Distance> tmp(vertices);
    for (size_t i = 0; i < landmarks.landmarks.size(); ++i) {
        const size_t offset = i * vertices;
        one_to_all(graph, static_cast<VertexId>(landmarks.landmarks[i]), tmp);
        std::copy(tmp.begin(), tmp.end(), landmarks.dist_from.begin() + static_cast<std::ptrdiff_t>(offset));

        one_to_all(reverse, static_cast<VertexId>(landmarks.landmarks[i]), tmp);
        std::copy(tmp.begin(), tmp.end(), landmarks.dist_to.begin() + static_cast<std::ptrdiff_t>(offset));
    }

    return landmarks;
}

} // namespace transport::alt
