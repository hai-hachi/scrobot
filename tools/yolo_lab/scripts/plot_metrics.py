#!/usr/bin/env python3
"""Plot YOLO training metrics from Ultralytics results.csv."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


METRIC_KEYS = {
    "Precision": "metrics/precision(B)",
    "Recall": "metrics/recall(B)",
    "mAP@50": "metrics/mAP50(B)",
    "mAP@50-95": "metrics/mAP50-95(B)",
}


def load_results(csv_path: Path):
    with csv_path.open("r", newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise RuntimeError(f"No rows found in {csv_path}")

    epochs = [int(float(row["epoch"])) + 1 for row in rows]
    series = {}

    for label, key in METRIC_KEYS.items():
        if key not in rows[0]:
            raise KeyError(
                f"Column '{key}' not found in {csv_path}.\n"
                f"Available columns: {', '.join(rows[0].keys())}"
            )
        series[label] = [float(row[key]) for row in rows]

    return epochs, series


def plot_metrics(csv_path: Path, output_path: Path, show: bool = False) -> None:
    epochs, series = load_results(csv_path)

    plt.figure(figsize=(10, 6))

    for label, values in series.items():
        plt.plot(epochs, values, marker="o", markersize=3, linewidth=1.8, label=label)

    plt.xlabel("Epoch")
    plt.ylabel("Metric")
    plt.title("Shuttlecock detector training metrics")
    plt.ylim(0.0, 1.0)
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=200)

    print(f"Metrics plot saved to: {output_path}")

    if show:
        plt.show()

    plt.close()


def main() -> None:
    p = argparse.ArgumentParser(description="Plot Ultralytics YOLO training metrics.")
    p.add_argument(
        "run",
        help="Run directory or results.csv, e.g. runs/detect/original_v5_continue",
    )
    p.add_argument("--show", action="store_true", help="Also open the plot window")
    args = p.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    path = Path(args.run).expanduser()
    if not path.is_absolute():
        path = repo_root / path

    if path.is_dir():
        csv_path = path / "results.csv"
        output_path = path / "metrics.png"
    else:
        csv_path = path
        output_path = path.with_name("metrics.png")

    if not csv_path.is_file():
        raise FileNotFoundError(f"results.csv not found: {csv_path}")

    plot_metrics(csv_path, output_path, show=args.show)


if __name__ == "__main__":
    main()
