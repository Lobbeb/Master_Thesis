#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import defaultdict
from pathlib import Path
from statistics import mean, median, stdev


DEFAULT_RESULTS_ROOT = Path("/home/ruben/halmstad_ws/models/results")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze Ultralytics metrics.json files saved under results/model_name/dataset_name/val/"
    )

    parser.add_argument(
        "--results",
        default=str(DEFAULT_RESULTS_ROOT),
        help="Results root folder.",
    )

    parser.add_argument(
        "--metric",
        default="map50_95",
        choices=["map50_95", "map50", "precision", "recall", "fitness"],
        help="Primary metric used for ranking models.",
    )

    parser.add_argument(
        "--output",
        default=None,
        help="Output folder for CSV summaries. Default: results/analysis",
    )

    parser.add_argument(
        "--show",
        type=int,
        default=30,
        help="Number of rows to print in terminal tables.",
    )

    return parser.parse_args()


def to_float(value):
    try:
        value = float(value)
        if math.isnan(value):
            return None
        return value
    except Exception:
        return None


def find_metric(metrics: dict, wanted: str):
    """
    Supports Ultralytics keys like:
      metrics/precision(B)
      metrics/recall(B)
      metrics/mAP50(B)
      metrics/mAP50-95(B)
      fitness
    """
    items = [(str(k).lower(), v) for k, v in metrics.items()]

    if wanted == "map50_95":
        for key, value in items:
            if "map50-95" in key or "map_50-95" in key:
                return to_float(value)

    elif wanted == "map50":
        for key, value in items:
            if "map50" in key and "map50-95" not in key:
                return to_float(value)

    elif wanted == "precision":
        for key, value in items:
            if "precision" in key:
                return to_float(value)

    elif wanted == "recall":
        for key, value in items:
            if "recall" in key:
                return to_float(value)

    elif wanted == "fitness":
        for key, value in items:
            if key == "fitness" or "fitness" in key:
                return to_float(value)

    return None


def parse_dataset_name(dataset_name: str):
    """
    road_to_east_v2 -> road_to_east, 2
    strip_v8        -> strip, 8
    """
    match = re.match(r"^(.*)_v(\d+)$", dataset_name)
    if not match:
        return dataset_name, ""
    return match.group(1), int(match.group(2))


def load_metrics_file(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # Your current format:
    # {
    #   "dataset": "...",
    #   "model": "...",
    #   "metrics": {...}
    # }
    metrics = data.get("metrics", data.get("results", data))

    dataset_name = data.get("dataset")
    model_file = data.get("model")

    # Fallback based on:
    # results/model_name/dataset_name/val/metrics.json
    if not dataset_name:
        dataset_name = path.parent.parent.name

    if not model_file:
        model_file = path.parent.parent.parent.name

    model_name = Path(model_file).stem
    scenario, version = parse_dataset_name(dataset_name)

    row = {
        "model": model_name,
        "model_file": model_file,
        "dataset": dataset_name,
        "scenario": scenario,
        "version": version,
        "metrics_json": str(path),
        "results_dir": str(path.parent),
        "map50_95": find_metric(metrics, "map50_95"),
        "map50": find_metric(metrics, "map50"),
        "precision": find_metric(metrics, "precision"),
        "recall": find_metric(metrics, "recall"),
        "fitness": find_metric(metrics, "fitness"),
    }

    return row


def load_all_runs(results_root: Path) -> list[dict]:
    rows = []

    for path in sorted(results_root.glob("*/*/val/metrics.json")):
        try:
            rows.append(load_metrics_file(path))
        except Exception as exc:
            print(f"Could not read {path}: {exc}")

    return rows


def numeric_values(rows: list[dict], key: str) -> list[float]:
    values = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, (int, float)) and not math.isnan(value):
            values.append(float(value))
    return values


def summarize_by_model(rows: list[dict], metric: str) -> list[dict]:
    grouped = defaultdict(list)

    for row in rows:
        grouped[row["model"]].append(row)

    summary = []

    for model, model_rows in grouped.items():
        primary = numeric_values(model_rows, metric)
        map50_95 = numeric_values(model_rows, "map50_95")
        map50 = numeric_values(model_rows, "map50")
        precision = numeric_values(model_rows, "precision")
        recall = numeric_values(model_rows, "recall")

        if not primary:
            continue

        primary_std = stdev(primary) if len(primary) > 1 else 0.0

        summary.append(
            {
                "model": model,
                "runs": len(model_rows),
                f"mean_{metric}": mean(primary),
                f"median_{metric}": median(primary),
                f"std_{metric}": primary_std,
                f"min_{metric}": min(primary),
                f"max_{metric}": max(primary),
                "stability_mean_minus_std": mean(primary) - primary_std,
                "mean_map50_95": mean(map50_95) if map50_95 else "",
                "mean_map50": mean(map50) if map50 else "",
                "mean_precision": mean(precision) if precision else "",
                "mean_recall": mean(recall) if recall else "",
            }
        )

    summary.sort(key=lambda r: r[f"mean_{metric}"], reverse=True)
    return summary


def best_per_dataset(rows: list[dict], metric: str) -> list[dict]:
    grouped = defaultdict(list)

    for row in rows:
        if isinstance(row.get(metric), (int, float)):
            grouped[row["dataset"]].append(row)

    winners = []

    for dataset, dataset_rows in grouped.items():
        best = max(dataset_rows, key=lambda r: r[metric])
        winners.append(
            {
                "dataset": dataset,
                "scenario": best["scenario"],
                "version": best["version"],
                "best_model": best["model"],
                metric: best[metric],
                "map50_95": best["map50_95"],
                "map50": best["map50"],
                "precision": best["precision"],
                "recall": best["recall"],
                "metrics_json": best["metrics_json"],
            }
        )

    winners.sort(key=lambda r: (str(r["scenario"]), int(r["version"]) if r["version"] != "" else 0))
    return winners


def win_counts(winners: list[dict]) -> list[dict]:
    counts = defaultdict(int)

    for row in winners:
        counts[row["best_model"]] += 1

    rows = [{"model": model, "wins": count} for model, count in counts.items()]
    rows.sort(key=lambda r: r["wins"], reverse=True)
    return rows


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    if not rows:
        return

    fieldnames = list(rows[0].keys())

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def fmt(value) -> str:
    if isinstance(value, float):
        return f"{value:.4f}"
    return str(value)


def print_table(title: str, rows: list[dict], columns: list[str], limit: int) -> None:
    print()
    print(title)
    print("-" * len(title))

    if not rows:
        print("No rows.")
        return

    shown = rows[:limit]

    widths = {
        col: max(len(col), max(len(fmt(row.get(col, ""))) for row in shown))
        for col in columns
    }

    print("  ".join(col.ljust(widths[col]) for col in columns))
    print("  ".join("-" * widths[col] for col in columns))

    for row in shown:
        print("  ".join(fmt(row.get(col, "")).ljust(widths[col]) for col in columns))

    if len(rows) > limit:
        print(f"... {len(rows) - limit} more")


def main() -> int:
    args = parse_args()

    results_root = Path(args.results).expanduser().resolve()
    output_root = Path(args.output).expanduser().resolve() if args.output else results_root / "analysis"

    rows = load_all_runs(results_root)

    if not rows:
        print(f"No metrics.json files found under: {results_root}")
        print("Expected structure:")
        print("  results/model_name/dataset_name/val/metrics.json")
        return 1

    model_summary = summarize_by_model(rows, args.metric)
    dataset_winners = best_per_dataset(rows, args.metric)
    wins = win_counts(dataset_winners)

    write_csv(output_root / "all_validation_runs.csv", rows)
    write_csv(output_root / "model_summary.csv", model_summary)
    write_csv(output_root / "best_model_per_dataset.csv", dataset_winners)
    write_csv(output_root / "model_win_counts.csv", wins)

    print(f"Loaded runs: {len(rows)}")
    print(f"Primary metric: {args.metric}")
    print(f"Saved CSV files to: {output_root}")

    print_table(
        "Model summary",
        model_summary,
        [
            "model",
            "runs",
            f"mean_{args.metric}",
            f"std_{args.metric}",
            "stability_mean_minus_std",
            "mean_map50_95",
            "mean_map50",
            "mean_precision",
            "mean_recall",
        ],
        args.show,
    )

    print_table(
        "Win counts",
        wins,
        ["model", "wins"],
        args.show,
    )

    print_table(
        "Best model per dataset",
        dataset_winners,
        [
            "dataset",
            "best_model",
            args.metric,
            "map50_95",
            "map50",
            "precision",
            "recall",
        ],
        args.show,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())