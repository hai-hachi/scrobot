#!/usr/bin/env python3
"""Fine-tune the current shuttlecock detector for SCROBOT low-angle images."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from ultralytics import YOLO

from plot_metrics import plot_metrics


REPO_ROOT = Path(__file__).resolve().parent.parent


def repo_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Fine-tune shuttlecock YOLO weights.")
    p.add_argument("--model", default="best.pt", help="Starting weights")
    p.add_argument(
        "--data",
        default="dataset/gazebo_scrobot_simple/data.yaml",
        help="YOLO dataset YAML",
    )
    p.add_argument("--epochs", type=int, default=50)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=8)
    p.add_argument("--device", default=None, help="e.g. 0, cpu, mps")
    p.add_argument("--lr0", type=float, default=0.001)
    p.add_argument("--patience", type=int, default=15)
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--name", default="scrobot_finetune")
    p.add_argument("--optimizer", default="AdamW", help="AdamW, SGD, auto, ...")
    p.add_argument("--cos-lr", action="store_true", help="Use cosine LR schedule")
    p.add_argument(
        "--project",
        default="runs/detect",
        help="Ultralytics run directory root",
    )
    p.add_argument(
        "--artifacts-dir",
        default="artifacts/models",
        help="Stable location where best.pt is copied",
    )
    p.add_argument("--smoke-test", action="store_true", help="Train only 3 epochs")
    return p.parse_args()


def print_validation(metrics) -> None:
    print("\n" + "=" * 64)
    print("VALIDATION RESULT")
    print("=" * 64)
    print(f"Precision      : {metrics.box.mp:.4f}")
    print(f"Recall         : {metrics.box.mr:.4f}")
    print(f"mAP@50         : {metrics.box.map50:.4f}")
    print(f"mAP@75         : {metrics.box.map75:.4f}")
    print(f"mAP@50-95      : {metrics.box.map:.4f}")
    print(f"Fitness        : {metrics.box.fitness():.4f}")
    if getattr(metrics, "speed", None):
        print("\nSpeed (ms/image)")
        for key, value in metrics.speed.items():
            print(f"  {key:<12}: {value:.2f}")
    print("=" * 64)


def main() -> None:
    args = parse_args()

    model_path = repo_path(args.model)
    data_path = repo_path(args.data)
    project_dir = repo_path(args.project).resolve()
    artifacts_dir = repo_path(args.artifacts_dir).resolve()

    if not model_path.is_file():
        raise FileNotFoundError(f"Model not found: {model_path}")
    if not data_path.is_file():
        raise FileNotFoundError(f"Dataset YAML not found: {data_path}")

    epochs = 3 if args.smoke_test else args.epochs

    print("=" * 64)
    print("SCROBOT shuttle detector fine-tuning")
    print(f"model      : {model_path}")
    print(f"data       : {data_path}")
    print(f"epochs     : {epochs}")
    print(f"imgsz      : {args.imgsz}")
    print(f"batch      : {args.batch}")
    print(f"optimizer  : {args.optimizer}")
    print(f"lr0        : {args.lr0}")
    print(f"cos_lr     : {args.cos_lr}")
    print(f"device     : {args.device or 'auto'}")
    print(f"project    : {project_dir}")
    print("=" * 64)

    model = YOLO(str(model_path))

    train_args = dict(
        data=str(data_path),
        epochs=epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        lr0=args.lr0,
        patience=args.patience,
        workers=args.workers,
        project=str(project_dir),
        name=args.name,
        optimizer=args.optimizer,
        cos_lr=args.cos_lr,
        exist_ok=True,
        seed=42,
        close_mosaic=10,
        plots=True,
        verbose=True,
    )
    if args.device is not None:
        train_args["device"] = args.device

    results = model.train(**train_args)

    save_dir = Path(results.save_dir)
    best_path = save_dir / "weights" / "best.pt"
    results_csv = save_dir / "results.csv"
    metrics_png = save_dir / "metrics.png"

    print("\nTraining complete.")
    print(f"Ultralytics best weights: {best_path}")

    artifacts_dir.mkdir(parents=True, exist_ok=True)
    stable_best = artifacts_dir / f"{args.name}.pt"
    shutil.copy2(best_path, stable_best)

    latest_file = artifacts_dir.parent / "latest_model.txt"
    latest_file.parent.mkdir(parents=True, exist_ok=True)
    latest_file.write_text(str(stable_best.resolve()) + "\n")

    print(f"Stable model copy      : {stable_best}")
    print(f"Latest-model pointer   : {latest_file}")

    if results_csv.is_file():
        plot_metrics(results_csv, metrics_png)

    print("\nRunning validation on best.pt ...")
    best_model = YOLO(str(best_path))
    val_args = dict(
        data=str(data_path),
        split="val",
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        plots=True,
        verbose=True,
    )
    if args.device is not None:
        val_args["device"] = args.device

    metrics = best_model.val(**val_args)
    print_validation(metrics)

    rel_stable = stable_best.relative_to(REPO_ROOT)
    print("\nNext commands:")
    print(f"  python scripts/test_shuttle.py {rel_stable}")
    print(
        "  python scripts/validate_model.py latest "
        f"--data {data_path.relative_to(REPO_ROOT)} --split test "
        f"--device {args.device or '0'}"
    )


if __name__ == "__main__":
    main()
