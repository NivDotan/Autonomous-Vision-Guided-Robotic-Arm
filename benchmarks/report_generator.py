"""
Benchmark report generator.

Reads JSON results from pick_place_test, latency_profiler, and tracking_accuracy,
and generates a Markdown + matplotlib summary.

Usage:
    python benchmarks/report_generator.py --results results/ --output report.md
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def load_latest_json(directory: Path, prefix: str) -> dict | None:
    files = sorted(directory.glob(f"{prefix}*.json"), reverse=True)
    if not files:
        return None
    with open(files[0]) as f:
        return json.load(f)


def generate_markdown(
    pick_place: dict | None,
    latency: dict | None,
    tracking: dict | None,
    output_path: str,
) -> None:
    lines = [
        "# Robot SAM2 App — Benchmark Report\n",
        f"*Auto-generated*\n",
    ]

    if pick_place:
        n_success = sum(1 for r in pick_place if r.get("success"))
        n_total   = len(pick_place)
        rate      = 100 * n_success / max(1, n_total)
        avg_time  = sum(r.get("duration_s", 0) for r in pick_place) / max(1, n_total)
        lines += [
            "## Pick-and-Place Success Rate\n",
            f"| Metric | Value |\n|---|---|\n",
            f"| Trials | {n_total} |\n",
            f"| Successes | {n_success} |\n",
            f"| Success rate | **{rate:.1f}%** |\n",
            f"| Avg duration | {avg_time:.2f} s |\n\n",
        ]

    if latency:
        summary = latency.get("summary", {})
        lines += [
            "## Pipeline Latency\n",
            "| Stage | Mean (ms) | P95 (ms) | P99 (ms) |\n|---|---|---|---|\n",
        ]
        for stage, s in summary.items():
            lines.append(
                f"| {stage} | {s['mean_ms']:.2f} | {s['p95_ms']:.2f} | {s['p99_ms']:.2f} |\n"
            )
        lines.append("\n")

    if tracking:
        s = tracking.get("summary", {})
        lines += [
            "## Tracking Accuracy vs ArUco Ground Truth\n",
            f"| Metric | Value |\n|---|---|\n",
            f"| Mean error | {s.get('mean_px', 0):.2f} px |\n",
            f"| Median error | {s.get('median_px', 0):.2f} px |\n",
            f"| P95 error | {s.get('p95_px', 0):.2f} px |\n",
            f"| Max error | {s.get('max_px', 0):.2f} px |\n\n",
        ]

    # Try to generate plots.
    try:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(12, 4))

        if latency and latency.get("summary"):
            stages = list(latency["summary"].keys())
            means  = [latency["summary"][s]["mean_ms"] for s in stages]
            axes[0].barh(stages, means, color="#7dd3fc")
            axes[0].set_title("Pipeline Latency (mean ms)")
            axes[0].set_xlabel("milliseconds")

        if tracking and tracking.get("errors"):
            errs = [e["error_px"] for e in tracking["errors"]]
            axes[1].hist(errs, bins=30, color="#22c55e", edgecolor="#0f1117")
            axes[1].set_title("Tracking Error Distribution")
            axes[1].set_xlabel("pixels")
            axes[1].set_ylabel("count")

        plot_path = str(Path(output_path).parent / "benchmark_plots.png")
        plt.tight_layout()
        plt.savefig(plot_path, dpi=150)
        plt.close()
        lines.append(f"![Benchmark Plots](benchmark_plots.png)\n")
        print(f"Plots saved to {plot_path}")
    except ImportError:
        lines.append("*(matplotlib not installed — plots skipped)*\n")

    with open(output_path, "w") as f:
        f.writelines(lines)
    print(f"Report saved to {output_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=str, default="results")
    parser.add_argument("--output",  type=str, default="results/report.md")
    args = parser.parse_args()

    results_dir = Path(args.results)
    pick_place  = load_latest_json(results_dir, "pick_place_")
    latency     = load_latest_json(results_dir, "latency_")
    tracking    = load_latest_json(results_dir, "tracking_accuracy_")

    if not any([pick_place, latency, tracking]):
        print("No benchmark results found. Run the individual benchmarks first.")
        return

    generate_markdown(
        pick_place.get("trials") if pick_place else None,
        latency,
        tracking,
        args.output,
    )


if __name__ == "__main__":
    main()
