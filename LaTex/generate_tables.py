#!/usr/bin/env python3
"""Generate LaTeX table fragments from bench_replay JSON results."""

import argparse
import glob
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path

EXPECTED_COUNT = 1000
POLAND_RANGED = "poland_ranged_q1000_s1.json"
EUROPE_RANGED = "europe_ranged_q1000_s1.json"
POLAND_RANDOM = "poland_random_q1000_s2.json"

ALGO_ORDER = ["dijkstra", "astar", "bidijkstra", "bidi_astar", "alt", "ch", "arcflags", "chase", "hl"]
ALGO_NICE = {
    "dijkstra": r"Dijkstra",
    "astar": r"A*",
    "bidijkstra": r"Bidirectional Dijkstra",
    "bidi_astar": r"Bidirectional A*",
    "alt": r"ALT (16 landmarks)",
    "ch": r"CH",
    "arcflags": r"Arc Flags (32 regions)",
    "chase": r"CHASE (64 regions)",
    "hl": r"Hub Labels (0.05)",
}

PINNED_MAIN_VARIANTS = {
    "alt": "farthest-16-active-4",
    "arcflags": "32-partitions-inertial",
}


@dataclass(frozen=True)
class Result:
    path: str
    data: dict

    @property
    def basename(self) -> str:
        return os.path.basename(self.path)

    @property
    def algorithm(self) -> str:
        return self.data.get("algorithm", "")

    @property
    def variant(self) -> str:
        return str(self.data.get("variant", ""))

    @property
    def date(self) -> str:
        return str(self.data.get("date", ""))

    @property
    def query_set_basename(self) -> str:
        return os.path.basename(str(self.data.get("query_set", "")))


def tex_escape(value: object) -> str:
    s = str(value)
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(ch, ch) for ch in s)


def fmt_s(v, decimals=1):
    if v == 0.0 or v < 0.001:
        return r"$<\!0.001$"
    return f"{v:,.{decimals}f}".replace(",", r"\,")


def fmt_minutes(seconds):
    return f"{seconds / 60.0:,.1f}".replace(",", r"\,") + r"\,min"


def fmt_us(v):
    if v < 1:
        return f"{v:.2f}"
    if v < 1000:
        return f"{v:.1f}"
    return f"{v:,.0f}".replace(",", r"\,")


def fmt_mb(v):
    if v == 0.0:
        return "---"
    if v >= 1024:
        return f"{v / 1024:.1f}".replace(",", r"\,") + r"\,GB"
    return f"{v:.0f}".replace(",", r"\,") + r"\,MB"


def load_results(results_dir):
    results = []
    for path in sorted(glob.glob(os.path.join(results_dir, "*.json"))):
        try:
            with open(path) as f:
                d = json.load(f)
        except Exception as exc:
            print(f"  skip {path}: {exc}", file=sys.stderr)
            continue
        if "algorithm" not in d or "algorithm_a" in d:
            continue
        if d.get("queries", {}).get("count", 0) != EXPECTED_COUNT:
            continue
        results.append(Result(path=path, data=d))
    return results


def is_validated_ranged(result, query_set):
    q = result.data.get("queries", {})
    return (
        result.query_set_basename == query_set
        and q.get("distance_validated") is True
        and q.get("distance_mismatches", 0) == 0
    )


def select_latest(results, predicate):
    matches = [r for r in results if predicate(r)]
    if not matches:
        return None
    return max(matches, key=lambda r: (r.date, r.basename))


def select_main_results(results, query_set):
    selected = {}
    for algo in ALGO_ORDER:
        pinned = PINNED_MAIN_VARIANTS.get(algo)
        result = select_latest(
            results,
            lambda r, algo=algo, pinned=pinned: r.algorithm == algo
            and is_validated_ranged(r, query_set)
            and (pinned is None or r.variant == pinned),
        )
        if result is not None:
            selected[algo] = result
    return selected


def preprocess_table(results, omit_missing=False):
    rows = [
        r"  \toprule",
        r"  \textbf{Algorithm} & \textbf{Dep.\ wall\,(s)} & \textbf{Dep.\ CPU\,(s)}"
        r" & \textbf{Algo\ wall\,(s)} & \textbf{Algo\ CPU\,(s)}"
        r" & \textbf{RSS\ after} & \textbf{Notes} \\",
        r"  \midrule",
    ]

    for algo in ALGO_ORDER:
        nice = ALGO_NICE.get(algo, algo)
        if algo not in results:
            if not omit_missing:
                rows.append(f"  {nice} & \\multicolumn{{6}}{{l}}{{---}} \\\\")
            continue

        pre = results[algo].data.get("preprocessing", {})
        notes = []
        if pre.get("ch_loaded_from"):
            notes.append("CH loaded")
        if pre.get("ch_saved_to"):
            notes.append("CH saved")
        if pre.get("arcflags_loaded_from"):
            notes.append("flags loaded")
        if pre.get("arcflags_saved_to"):
            notes.append("flags saved")
        if "auxiliary_edges" in pre:
            notes.append(f"+{pre['auxiliary_edges'] // 1_000_000}M aux.~edges")
        note_str = ", ".join(notes) if notes else "---"

        rows.append(
            f"  {nice} & {fmt_s(pre.get('dependency_wall_s', 0.0))} & {fmt_s(pre.get('dependency_cpu_s', 0.0))}"
            f" & {fmt_s(pre.get('algorithm_wall_s', 0.0))} & {fmt_s(pre.get('algorithm_cpu_s', 0.0))}"
            f" & {fmt_mb(pre.get('after_algorithm_peak_rss_mb', 0.0))} & {note_str} \\\\"
        )

    rows.append(r"  \bottomrule")
    return "\\begin{tabular}{lrrrrrl}\n" + "\n".join(rows) + "\n\\end{tabular}"


def query_table(results, dijkstra_mean, omit_missing=False, nice_overrides=None):
    nice_overrides = nice_overrides or {}
    rows = [
        r"  \toprule",
        r"  \textbf{Algorithm} & \textbf{Mean\,({\textmu}s)} & \textbf{P50} & \textbf{P95}"
        r" & \textbf{P99} & \textbf{Mean\ CPU\,({\textmu}s)} & \textbf{Speedup$^\dagger$} & \textbf{Validated} \\",
        r"  \midrule",
    ]

    for algo in ALGO_ORDER:
        nice = nice_overrides.get(algo, ALGO_NICE.get(algo, algo))
        if algo not in results:
            if not omit_missing:
                rows.append(f"  {nice} & \\multicolumn{{7}}{{l}}{{---}} \\\\")
            continue
        q = results[algo].data.get("queries", {})
        mean_w = q.get("mean_wall_us", 0)
        if dijkstra_mean and mean_w > 0:
            sp = dijkstra_mean / mean_w
            speedup = (f"{sp:,.0f}".replace(",", r"\,") if sp >= 100 else f"{sp:.1f}") + r"$\times$"
        else:
            speedup = "---"
        validated = r"$\checkmark$" if q.get("distance_validated") and q.get("distance_mismatches", 0) == 0 else "---"
        rows.append(
            f"  {nice} & {fmt_us(mean_w)} & {fmt_us(q.get('p50_wall_us', 0))} & {fmt_us(q.get('p95_wall_us', 0))}"
            f" & {fmt_us(q.get('p99_wall_us', 0))} & {fmt_us(q.get('mean_cpu_us', 0))}"
            f" & {speedup} & {validated} \\\\"
        )

    rows.append(r"  \bottomrule")
    return "\\begin{tabular}{lrrrrrrl}\n" + "\n".join(rows) + "\n\\end{tabular}"


def write_main_tables(graph_key, results, outpath, query_caption, omit_missing=False):
    dijkstra_mean = None
    if "dijkstra" in results:
        dijkstra_mean = results["dijkstra"].data.get("queries", {}).get("mean_wall_us")

    lines = [
        "% Auto-generated by LaTex/generate_tables.py -- do not edit by hand",
        "",
        r"\subsection*{Preprocessing}",
        "",
        r"\begin{table}[ht]",
        r"\centering",
        r"\caption{Preprocessing times and peak RSS --- " + graph_key.title() + r" graph.}",
        r"\label{tab:preprocess-" + graph_key + r"}",
        r"\resizebox{\textwidth}{!}{" + preprocess_table(results, omit_missing=omit_missing) + "}",
        r"\end{table}",
        "",
        r"\subsection*{Query performance}",
        "",
        r"\begin{table}[ht]",
        r"\centering",
        r"\caption{Query latency --- " + query_caption + r"}",
        r"\label{tab:query-" + graph_key + r"}",
        r"\resizebox{\textwidth}{!}{" + query_table(results, dijkstra_mean, omit_missing=omit_missing) + "}",
        r"\noindent\footnotesize $^\dagger$Speedup = Dijkstra mean wall / algorithm mean wall.\\",
        r"\end{table}",
        "",
    ]
    write_file(outpath, lines)


def prune_rate(result):
    q = result.data.get("queries", {})
    relaxed = q.get("mean_relaxed_arcs", 0.0)
    if relaxed <= 0:
        return 0.0
    return 100.0 * q.get("mean_pruned_by_flag", 0.0) / relaxed


def select_arcflags_sweep(results):
    specs = [
        (
            "Inertial-32",
            1,
            lambda r: r.algorithm == "arcflags"
            and r.variant == "32-partitions-inertial"
            and is_validated_ranged(r, POLAND_RANGED)
            and r.data.get("preprocessing", {}).get("algorithm_cpu_s", 0.0) < 10000.0,
        ),
        (
            "Inertial-32",
            16,
            lambda r: r.algorithm == "arcflags"
            and r.variant == "32-partitions-inertial"
            and is_validated_ranged(r, POLAND_RANGED)
            and r.data.get("preprocessing", {}).get("algorithm_cpu_s", 0.0) >= 10000.0,
        ),
        (
            "Inertial-64",
            16,
            lambda r: r.algorithm == "arcflags"
            and r.variant in {"64-partitions-inertial", "Inertial partition, 64 regions, 16 threads"}
            and is_validated_ranged(r, POLAND_RANGED),
        ),
        (
            "KaMinPar-64",
            16,
            lambda r: r.algorithm == "arcflags"
            and r.variant == "64-partitions-kaminpar"
            and is_validated_ranged(r, POLAND_RANGED),
        ),
    ]
    rows = []
    for label, threads, pred in specs:
        result = select_latest(results, pred)
        rows.append((label, threads, result))
    return rows


def write_arcflags_sweep(rows, outpath, dijkstra_mean):
    body = [
        r"\begin{table}[ht]",
        r"\centering",
        r"\caption{Arc Flags configuration sweep --- Poland graph, 1\,000 ranged queries, seed~1.}",
        r"\label{tab:arcflags-sweep}",
    ]
    if not any(result is not None for _, _, result in rows):
        body += [r"\noindent No Arc Flags sweep result JSONs were found.", r"\end{table}"]
        write_file(outpath, ["% Auto-generated by LaTex/generate_tables.py -- do not edit by hand", *body, ""])
        return

    table_rows = [
        r"  \toprule",
        r"  \textbf{Variant} & \textbf{Threads} & \textbf{Pre.\ wall} & \textbf{Pre.\ CPU} & \textbf{RSS} &"
        r" \textbf{Mean\,(\textmu{}s)} & \textbf{P99\,(\textmu{}s)} & \textbf{Prune rate} & \textbf{Speedup} \\",
        r"  \midrule",
    ]
    missing_notes = []
    for label, threads, result in rows:
        if result is None:
            missing_notes.append(f"{label}: no result JSON available")
            table_rows.append(f"  {label} & {threads} & \\multicolumn{{7}}{{l}}{{not run}} \\\\")
            continue
        pre = result.data.get("preprocessing", {})
        q = result.data.get("queries", {})
        speedup = "---"
        if dijkstra_mean and q.get("mean_wall_us", 0) > 0:
            speedup = f"{dijkstra_mean / q['mean_wall_us']:.1f}" + r"$\times$"
        table_rows.append(
            f"  {label} & {threads} & {fmt_minutes(pre.get('algorithm_wall_s', 0.0))}"
            f" & {fmt_s(pre.get('algorithm_cpu_s', 0.0), 0)}\\,s & {fmt_mb(pre.get('after_algorithm_peak_rss_mb', 0.0))}"
            f" & {fmt_us(q.get('mean_wall_us', 0.0))} & {fmt_us(q.get('p99_wall_us', 0.0))}"
            f" & {prune_rate(result):.1f}\\% & {speedup} \\\\"
        )
    table_rows.append(r"  \bottomrule")
    body += [
        r"\resizebox{\textwidth}{!}{%",
        "\\begin{tabular}{lrrrrrrrr}",
        *table_rows,
        "\\end{tabular}}",
        r"\smallskip",
        r"\noindent\footnotesize Prune rate = mean pruned arcs / mean relaxed arcs per query.\\",
        (
            r"\noindent\footnotesize "
            + tex_escape("; ".join(missing_notes))
            + r".\\"
            if missing_notes
            else ""
        ),
        r"\end{table}",
    ]
    write_file(outpath, ["% Auto-generated by LaTex/generate_tables.py -- do not edit by hand", *body, ""])


def select_alt_strategy(results):
    labels = ["farthest-16-active-4", "random-16-active-4", "planar-16-active-4"]
    rows = []
    for label in labels:
        result = select_latest(
            results,
            lambda r, label=label: r.algorithm == "alt"
            and r.variant == label
            and is_validated_ranged(r, POLAND_RANGED),
        )
        if result is not None:
            rows.append((label, result))
    return rows


def write_alt_strategy(rows, outpath, dijkstra_mean):
    lines = [
        "% Auto-generated by LaTex/generate_tables.py -- do not edit by hand",
        r"\begin{table}[ht]",
        r"\centering",
        r"\caption{ALT landmark-selection strategies --- Poland graph, 1\,000 ranged queries, seed~1.}",
        r"\label{tab:alt-strategy}",
    ]
    if not rows:
        lines += [r"\noindent ALT landmark-selection benchmark JSONs were not available.", r"\end{table}", ""]
        write_file(outpath, lines)
        return
    table_rows = [
        r"  \toprule",
        r"  \textbf{Variant} & \textbf{Pre.\ wall\,(s)} & \textbf{RSS} & \textbf{Mean\,(\textmu{}s)}"
        r" & \textbf{P99\,(\textmu{}s)} & \textbf{Speedup} \\",
        r"  \midrule",
    ]
    for label, result in rows:
        pre = result.data.get("preprocessing", {})
        q = result.data.get("queries", {})
        speedup = "---"
        if dijkstra_mean and q.get("mean_wall_us", 0) > 0:
            speedup = f"{dijkstra_mean / q['mean_wall_us']:.1f}" + r"$\times$"
        table_rows.append(
            f"  {tex_escape(label)} & {fmt_s(pre.get('algorithm_wall_s', 0.0))}"
            f" & {fmt_mb(pre.get('after_algorithm_peak_rss_mb', 0.0))}"
            f" & {fmt_us(q.get('mean_wall_us', 0.0))} & {fmt_us(q.get('p99_wall_us', 0.0))} & {speedup} \\\\"
        )
    table_rows.append(r"  \bottomrule")
    lines += [
        r"\resizebox{\textwidth}{!}{%",
        "\\begin{tabular}{lrrrrr}",
        *table_rows,
        "\\end{tabular}}",
        r"\end{table}",
        "",
    ]
    write_file(outpath, lines)


def select_random_pair_results(results):
    selected = {}
    for algo in ALGO_ORDER:
        result = select_latest(
            results,
            lambda r, algo=algo: r.algorithm == algo
            and r.query_set_basename == POLAND_RANDOM
            and r.data.get("queries", {}).get("distance_validated") is False,
        )
        if result is not None:
            selected[algo] = result
    return selected


def write_random_pairs(results, outpath):
    lines = [
        "% Auto-generated by LaTex/generate_tables.py -- do not edit by hand",
        r"\begin{table}[ht]",
        r"\centering",
        r"\caption{Random-pair check --- Poland graph, 1\,000 random pairs, seed~2.}",
        r"\label{tab:random-pairs}",
    ]
    if not results:
        lines += [r"\noindent Random-pair benchmark JSONs were not available.", r"\end{table}", ""]
        write_file(outpath, lines)
        return
    dijkstra_mean = results.get("dijkstra").data.get("queries", {}).get("mean_wall_us") if "dijkstra" in results else None
    lines += [
        r"\resizebox{\textwidth}{!}{"
        + query_table(
            results,
            dijkstra_mean,
            omit_missing=True,
            nice_overrides={"arcflags": r"Arc Flags (KaMinPar-64)", "alt": r"ALT (farthest, 16/4)"},
        )
        + "}",
        r"\noindent\footnotesize Random pairs do not carry reference distances; \texttt{distance\_validated=false}.\\",
        r"\end{table}",
        "",
    ]
    write_file(outpath, lines)


def provenance_rows(groups):
    rows = []
    for group, items in groups:
        for result in items:
            rows.append((group, result))
    return rows


def write_result_sources(rows, outpath):
    lines = [
        "% Auto-generated by LaTex/generate_tables.py -- do not edit by hand",
        r"\section*{Result JSON Files Used}",
        r"\addcontentsline{toc}{section}{Result JSON Files Used}",
    ]
    if not rows:
        lines += [r"No benchmark result JSON files were selected by the table generator.", ""]
        write_file(outpath, lines)
        return
    table_rows = [
        r"  \toprule",
        r"  \textbf{Table group} & \textbf{JSON file} & \textbf{Commit} & \textbf{Variant}"
        r" & \textbf{Graph} & \textbf{Query set} & \textbf{Queries} \\",
        r"  \midrule",
    ]
    for group, result in rows:
        d = result.data
        graph = os.path.basename(str(d.get("graph", {}).get("path", "")))
        table_rows.append(
            f"  {tex_escape(group)} & {tex_escape(result.basename)} & {tex_escape(d.get('commit', ''))}"
            f" & {tex_escape(d.get('variant', ''))} & {tex_escape(graph)}"
            f" & {tex_escape(result.query_set_basename)} & {d.get('queries', {}).get('count', '')} \\\\"
        )
    table_rows.append(r"  \bottomrule")
    lines += [
        r"\small",
        r"\resizebox{\textwidth}{!}{%",
        "\\begin{tabular}{llllllr}",
        *table_rows,
        "\\end{tabular}}",
        r"\normalsize",
        "",
    ]
    write_file(outpath, lines)


def write_file(path, lines):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Wrote {path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", default="results")
    parser.add_argument("--out-dir", default="LaTex")
    args = parser.parse_args()

    results = load_results(args.results_dir)
    poland = select_main_results(results, POLAND_RANGED)
    europe = select_main_results(results, EUROPE_RANGED)
    arcflags_sweep = select_arcflags_sweep(results)
    alt_strategy = select_alt_strategy(results)
    random_pairs = select_random_pair_results(results)

    print(f"Selected Poland main results: {sorted(poland)}")
    print(f"Selected Europe main results: {sorted(europe)}")
    print(f"Selected Arc Flags sweep rows: {[label for label, _, result in arcflags_sweep if result is not None]}")
    print(f"Selected ALT strategy rows: {[label for label, _ in alt_strategy]}")
    print(f"Selected random-pair results: {sorted(random_pairs)}")

    poland_dijkstra = poland.get("dijkstra")
    dijkstra_mean = poland_dijkstra.data.get("queries", {}).get("mean_wall_us") if poland_dijkstra else None

    write_main_tables(
        "poland",
        poland,
        f"{args.out_dir}/tables_poland.tex",
        r"Poland graph, 1\,000 queries, seed~1, settled $\in[10^5,10^6]$.",
    )
    write_main_tables(
        "europe",
        europe,
        f"{args.out_dir}/tables_europe.tex",
        r"Europe graph, 1\,000 queries, seed~1, settled $\in[10^5,10^6]$.",
        omit_missing=True,
    )
    write_arcflags_sweep(arcflags_sweep, f"{args.out_dir}/tables_arcflags_sweep.tex", dijkstra_mean)
    write_alt_strategy(alt_strategy, f"{args.out_dir}/tables_alt_strategy.tex", dijkstra_mean)
    write_random_pairs(random_pairs, f"{args.out_dir}/tables_random_pairs.tex")

    groups = [
        ("Main Poland", poland.values()),
        ("Main Europe", europe.values()),
        ("Arc Flags sweep", [r for _, _, r in arcflags_sweep if r is not None]),
        ("ALT strategy", [r for _, r in alt_strategy]),
        ("Random pairs", random_pairs.values()),
    ]
    write_result_sources(provenance_rows(groups), f"{args.out_dir}/result_sources.tex")
    print("Done.")


if __name__ == "__main__":
    main()
