#include "algorithms/hl/hub_labels.hpp"

#include "algorithms/heap_node.hpp"
#include "algorithms/stopwatch.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <queue>
#include <span>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace transport {

namespace {

using Pq = std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<HeapNode>>;

// a + b, saturating at kUnreachable instead of wrapping around.
constexpr Distance saturating_add(Distance a, Distance b) { return a <= kUnreachable - b ? a + b : kUnreachable; }

} // namespace

HubLabelsAlgorithm::HubLabelsAlgorithm(const Graph &graph, double label_fraction, uint64_t memory_budget_bytes)
    : graph_(graph), label_fraction_(label_fraction), memory_budget_bytes_(memory_budget_bytes) {
    if (label_fraction_ <= 0.0 || label_fraction_ > 1.0 || !std::isfinite(label_fraction_)) {
        throw std::invalid_argument("hl: label_fraction must be in (0, 1]");
    }
    if (memory_budget_bytes_ == 0) {
        throw std::invalid_argument("hl: memory_budget_bytes must be > 0");
    }
}

std::string_view HubLabelsAlgorithm::name() const { return "hl"; }

void HubLabelsAlgorithm::inject_ch(ContractionHierarchy ch) {
    ch_ = std::move(ch);
    ch_provided_ = true;
}

// ----- label intersection -----

Distance HubLabelsAlgorithm::intersect_labels(std::span<const HlEntry> f, std::span<const HlEntry> b) {
    Distance best = kUnreachable;
    size_t i = 0;
    size_t j = 0;
    while (i < f.size() && j < b.size()) {
        if (f[i].hub == b[j].hub) {
            best = std::min(best, saturating_add(f[i].dist, b[j].dist));
            ++i;
            ++j;
        } else if (f[i].hub < b[j].hub) {
            ++i;
        } else {
            ++j;
        }
    }
    return best;
}

// ----- preprocess -----

void HubLabelsAlgorithm::preprocess() {
    if (preprocessed_) {
        return;
    }

    if (!ch_provided_) {
        ContractionHierarchyAlgorithm ch_algo(graph_);
        ch_algo.preprocess();
        ch_ = ch_algo.get_ch();
    }
    stats_.ch_was_injected = ch_provided_;

    const VertexId V = ch_.vertex_count();
    labeled_count_ = static_cast<VertexId>(static_cast<double>(V) * label_fraction_);
    labeled_count_ = std::clamp(labeled_count_, VertexId{1}, V); // at least the apex
    label_threshold_ = V - labeled_count_;

    fwd_scratch_ = StampedVector<Distance>(V, kUnreachable);
    bwd_scratch_ = StampedVector<Distance>(V, kUnreachable);

    build_labels();

    stats_.label_fraction = label_fraction_;
    stats_.labeled_vertices = static_cast<uint32_t>(labeled_count_);
    preprocessed_ = true;
}

void HubLabelsAlgorithm::build_labels() {
    Stopwatch sw;
    const VertexId V = ch_.vertex_count();

    // inv_rank[r] = vertex with rank r.
    std::vector<VertexId> inv_rank(V);
    for (VertexId v = 0; v < V; ++v) {
        inv_rank[ch_.rank[v]] = v;
    }

    // Temporary per-vertex label storage indexed by vertex_id.
    // Unlabeled vertices keep empty vectors throughout.
    std::vector<std::vector<HlEntry>> temp_fwd(V);
    std::vector<std::vector<HlEntry>> temp_bwd(V);

    uint64_t prune_drops = 0;
    uint64_t total_fwd_entries = 0;
    uint64_t total_bwd_entries = 0;
    size_t max_fwd = 0;
    size_t max_bwd = 0;

    // Shared helper: builds one half-label for `vertex` over `out_edges` (the vertex's
    // upward arcs in one CH direction), propagating distances through already-built
    // `own_temp` labels and pruning against `other_temp`.
    auto build_half = [&](VertexId vertex, std::span<const Edge> out_edges, std::vector<std::vector<HlEntry>> &own_temp,
                          const std::vector<std::vector<HlEntry>> &other_temp) -> std::vector<HlEntry> & {
        std::unordered_map<VertexId, Distance> cand_map;
        cand_map[vertex] = Distance{0};
        for (const Edge &e : out_edges) {
            for (const HlEntry &he : own_temp[e.to]) {
                const Distance nd = saturating_add(he.dist, e.weight);
                auto [it, inserted] = cand_map.emplace(he.hub, nd);
                if (!inserted && nd < it->second) {
                    it->second = nd;
                }
            }
        }
        std::vector<HlEntry> sorted_cands;
        sorted_cands.reserve(cand_map.size());
        for (const auto &[h, d] : cand_map) {
            sorted_cands.push_back({h, d});
        }
        std::sort(sorted_cands.begin(), sorted_cands.end());

        std::vector<HlEntry> &label = own_temp[vertex];
        label.clear();
        for (const HlEntry &cand : sorted_cands) {
            if (intersect_labels(std::span(label), std::span<const HlEntry>(other_temp[cand.hub])) < cand.dist) {
                ++prune_drops;
            } else {
                label.push_back(cand);
            }
        }
        return label;
    };

    // Fixed overhead for the memory-budget projection below; independent of the loop
    // iteration, so computed once. Covers CH arcs + graph edges + per-vertex: temp vector
    // headers (2×), inv_rank, fwd/bwd scratch (2×), fwd/bwd offsets (2×). Element sizes are
    // taken via sizeof(container[0]) (unevaluated, so safe on empty containers) so the
    // estimate stays correct if any underlying type changes.
    const uint64_t fixed_bytes =
        (ch_.forward_edges.size() + ch_.backward_edges.size()) * sizeof(ch_.forward_edges[0]) +
        (ch_.forward_offsets.size() + ch_.backward_offsets.size()) * sizeof(ch_.forward_offsets[0]) +
        ch_.rank.size() * sizeof(ch_.rank[0]) + graph_.edges.size() * sizeof(graph_.edges[0]) +
        graph_.offsets.size() * sizeof(graph_.offsets[0]) +
        static_cast<uint64_t>(V) *
            (2 * sizeof(temp_fwd[0]) + sizeof(inv_rank[0]) + 2 * sizeof(Distance) + 2 * sizeof(fwd_offsets_[0]));

    // Process labeled vertices in descending rank order.
    for (VertexId rank = V; rank-- > label_threshold_;) {
        const VertexId v = inv_rank[rank];

        const std::vector<HlEntry> &lf = build_half(v, ch_.forward_adjacent_edges(v), temp_fwd, temp_bwd);
        const std::vector<HlEntry> &lb = build_half(v, ch_.backward_adjacent_edges(v), temp_bwd, temp_fwd);

        total_fwd_entries += lf.size();
        total_bwd_entries += lb.size();
        max_fwd = std::max(max_fwd, lf.size());
        max_bwd = std::max(max_bwd, lb.size());

        // Incremental memory budget check every 64k vertices.
        // Peak memory ≈ 2 × label_data (temp vectors + CSR copy coexist during assembly)
        //             + fixed overhead (CH arcs, graph edges, per-vertex scratch).
        const VertexId processed = V - rank;
        if ((processed & 0xFFFFu) == 0 && processed > 0) {
            const uint64_t bytes_so_far = (total_fwd_entries + total_bwd_entries) * sizeof(HlEntry);
            const uint64_t projected_label_bytes =
                bytes_so_far * static_cast<uint64_t>(labeled_count_) / static_cast<uint64_t>(processed);
            const uint64_t projected_peak = 2 * projected_label_bytes + fixed_bytes;
            if (projected_peak > memory_budget_bytes_) {
                throw std::runtime_error("hl: projected peak memory " +
                                         std::to_string(projected_peak / (1024ULL * 1024)) + " MB exceeds budget " +
                                         std::to_string(memory_budget_bytes_ / (1024ULL * 1024 * 1024)) +
                                         " GB after processing " + std::to_string(processed) + " of " +
                                         std::to_string(labeled_count_) + " labeled vertices");
            }
        }
    }

    // Assemble CSR from temp vectors.
    fwd_offsets_.resize(static_cast<size_t>(V) + 1);
    bwd_offsets_.resize(static_cast<size_t>(V) + 1);
    fwd_labels_.reserve(total_fwd_entries);
    bwd_labels_.reserve(total_bwd_entries);

    fwd_offsets_[0] = 0;
    bwd_offsets_[0] = 0;
    for (VertexId v = 0; v < V; ++v) {
        fwd_offsets_[v + 1] = fwd_offsets_[v] + temp_fwd[v].size();
        bwd_offsets_[v + 1] = bwd_offsets_[v] + temp_bwd[v].size();
        fwd_labels_.insert(fwd_labels_.end(), temp_fwd[v].begin(), temp_fwd[v].end());
        bwd_labels_.insert(bwd_labels_.end(), temp_bwd[v].begin(), temp_bwd[v].end());
        // Release memory as we go.
        temp_fwd[v] = std::vector<HlEntry>{};
        temp_bwd[v] = std::vector<HlEntry>{};
    }

    const uint64_t total_label_bytes = (total_fwd_entries + total_bwd_entries) * sizeof(HlEntry);

    stats_.label_build_wall_s = to_seconds(sw.wall_elapsed());
    stats_.label_build_cpu_s = to_seconds(sw.cpu_elapsed());
    stats_.avg_label_size_fwd =
        labeled_count_ > 0 ? static_cast<double>(total_fwd_entries) / static_cast<double>(labeled_count_) : 0.0;
    stats_.avg_label_size_bwd =
        labeled_count_ > 0 ? static_cast<double>(total_bwd_entries) / static_cast<double>(labeled_count_) : 0.0;
    stats_.max_label_size_fwd = max_fwd;
    stats_.max_label_size_bwd = max_bwd;
    stats_.label_memory_mb = static_cast<double>(total_label_bytes) / (1024.0 * 1024.0);
    stats_.prune_drops = prune_drops;
}

// ----- collect (shared upward-search helper) -----

uint32_t HubLabelsAlgorithm::collect(VertexId start, bool forward, std::vector<HlEntry> &out,
                                     std::vector<VertexId> *unlabeled) const {
    StampedVector<Distance> &scratch = forward ? fwd_scratch_ : bwd_scratch_;

    std::unordered_map<VertexId, Distance> hub_best;
    Pq pq;
    scratch.set(start, Distance{0});
    pq.push({Distance{0}, start});

    uint32_t settled = 0;
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != scratch.get(top.v)) {
            continue;
        }
        ++settled;
        if (is_labeled(top.v)) {
            for (const HlEntry &e : (forward ? fwd_label(top.v) : bwd_label(top.v))) {
                const Distance nd = saturating_add(top.key, e.dist);
                auto [it, inserted] = hub_best.emplace(e.hub, nd);
                if (!inserted && nd < it->second) {
                    it->second = nd;
                }
            }
            continue;
        }
        if (unlabeled != nullptr) {
            unlabeled->push_back(top.v);
        }
        for (const Edge &e : (forward ? ch_.forward_adjacent_edges(top.v) : ch_.backward_adjacent_edges(top.v))) {
            const Distance nd = saturating_add(top.key, e.weight);
            if (nd < scratch.get(e.to)) {
                scratch.set(e.to, nd);
                pq.push({nd, e.to});
            }
        }
    }

    out.clear();
    out.reserve(hub_best.size());
    for (const auto &[h, d] : hub_best) {
        out.push_back({h, d});
    }
    std::sort(out.begin(), out.end());
    return settled;
}

// ----- query -----

PathResult HubLabelsAlgorithm::query(VertexId source, VertexId target) const {
    if (!preprocessed_) {
        throw std::logic_error("HubLabelsAlgorithm::preprocess() must be called before query()");
    }

    const bool s_lab = is_labeled(source);
    const bool t_lab = is_labeled(target);
    const bool both_unlabeled = !s_lab && !t_lab;
    uint32_t settled = 0;

    // Forward side: source's stored hub label, or an upward search from source.
    // fwd_unlabeled is recorded only when the mu_low fallback below needs it.
    std::vector<HlEntry> fwd_buf;
    std::vector<VertexId> fwd_unlabeled;
    std::span<const HlEntry> fwd_side;
    if (s_lab) {
        fwd_side = fwd_label(source);
    } else {
        fwd_scratch_.reset();
        settled += collect(source, /*forward=*/true, fwd_buf, both_unlabeled ? &fwd_unlabeled : nullptr);
        fwd_side = fwd_buf;
    }

    // Backward side: target's stored hub label, or an upward search from target.
    std::vector<HlEntry> bwd_buf;
    std::span<const HlEntry> bwd_side;
    if (t_lab) {
        bwd_side = bwd_label(target);
    } else {
        bwd_scratch_.reset();
        settled += collect(target, /*forward=*/false, bwd_buf, nullptr);
        bwd_side = bwd_buf;
    }

    Distance dist = intersect_labels(fwd_side, bwd_side);

    // When both endpoints are unlabeled the apex may lie below the labeled tier; mu_low
    // captures those paths via the overlap of the two bidirectional CH searches.
    if (both_unlabeled) {
        for (const VertexId v : fwd_unlabeled) {
            const Distance db = bwd_scratch_.get(v);
            if (db != kUnreachable) {
                dist = std::min(dist, saturating_add(fwd_scratch_.get(v), db));
            }
        }
    }

    if (!s_lab) {
        fwd_scratch_.reset();
    }
    if (!t_lab) {
        bwd_scratch_.reset();
    }

    return {dist, settled};
}

} // namespace transport
