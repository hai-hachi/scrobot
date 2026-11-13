#!/usr/bin/env python3
"""Validate any shuttlecock YOLO model and print the main metrics."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


REPO_ROOT = Path(__file__).resolve().parent.parent


def repo_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


def resolve_model(value: str) -> Path:
    if value == "latest":
        pointer = REPO_ROOT / "artifacts/latest_model.txt"
        if pointer.is_file():
            candidate = Path(pointer.read_text().strip()).expanduser()
            if candidate.is_file():
                return candidate

        models_dir = REPO_ROOT / "artifacts/models"
        candidates = sorted(
            models_dir.glob("*.pt"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        ) if models_dir.is_dir() else []
        if candidates:
            return candidates[0]

        runs_dir = REPO_ROOT / "runs/detect"
        run_candidates = sorted(
            runs_dir.glob("*/weights/best.pt"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        ) if runs_dir.is_dir() else []
        if run_candidates:
            return run_candidates[0]

        raise FileNotFoundError(
            "No trained model found under artifacts/models or runs/detect."
        )

    direct = repo_path(value)
    if direct.is_file():
        return direct

    run_candidate = REPO_ROOT / "runs/detect" / value / "weights/best.pt"
    if run_candidate.is_file():
        return run_candidate

    artifact_candidate = REPO_ROOT / "artifacts/models" / f"{value}.pt"
    if artifact_candidate.is_file():
        return artifact_candidate

    available = []
    models_dir = REPO_ROOT / "artifacts/models"
    runs_dir = REPO_ROOT / "runs/detect"

    if models_dir.is_dir():
        available.extend(str(p.relative_to(REPO_ROOT)) for p in sorted(models_dir.glob("*.pt")))
    if runs_dir.is_dir():
        available.extend(
            str(p.relative_to(REPO_ROOT))
            for p in sorted(runs_dir.glob("*/weights/best.pt"))
        )

    message = [f"Model not found: {value}"]
    if available:
        message.append("Available trained models:")
        message.extend(f"  - {p}" for p in available)
    raise FileNotFoundError("\n".join(message))


def main() -> None:
    p = argparse.ArgumentParser(description="Validate a shuttlecock detector.")
    p.add_argument("model", nargs="?", default="latest")
    p.add_argument(
        "--data",
        default="dataset/gazebo_scrobot_simple/data.yaml",
    )
    p.add_argument("--device", default="0")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--split", choices=["val", "test"], default="val")
    args = p.parse_args()

    model_path = resolve_model(args.model)
    data_path = repo_path(args.data)

    if not data_path.is_file():
        raise FileNotFoundError(f"Dataset YAML not found: {data_path}")

    print(f"Model   : {model_path}")
    print(f"Dataset : {data_path}")

    model = YOLO(str(model_path))
    metrics = model.val(
        data=str(data_path),
        split=args.split,
        device=args.device,
        imgsz=args.imgsz,
        batch=args.batch,
        verbose=True,
    )

    print("\n" + "=" * 64)
    print(f"VALIDATION RESULT: {model_path}")
    print(f"Split            : {args.split}")
    print("=" * 64)
    print(f"Precision        : {metrics.box.mp:.4f}")
    print(f"Recall           : {metrics.box.mr:.4f}")
    print(f"mAP@50           : {metrics.box.map50:.4f}")
    print(f"mAP@75           : {metrics.box.map75:.4f}")
    print(f"mAP@50-95        : {metrics.box.map:.4f}")
    print(f"Fitness          : {metrics.box.fitness():.4f}")
    if getattr(metrics, "speed", None):
        print("\nSpeed (ms/image)")
        for key, value in metrics.speed.items():
            print(f"  {key:<12}: {value:.2f}")
    print("=" * 64)


if __name__ == "__main__":
    main()
