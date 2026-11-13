#!/usr/bin/env python3
"""Quick live test for a shuttlecock YOLO model."""

from __future__ import annotations

import argparse
import time

import cv2
from ultralytics import YOLO

from validate_model import resolve_model


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Live shuttlecock detector test.")
    p.add_argument("model", nargs="?", default="latest")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--conf", type=float, default=0.10)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    return p.parse_args()


def main() -> None:
    args = parse_args()
    model_path = resolve_model(args.model)
    print(f"Using model: {model_path}")
    model = YOLO(str(model_path))

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.camera}")

    fps_smooth = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            t0 = time.perf_counter()
            result = model.predict(
                frame,
                imgsz=args.imgsz,
                conf=args.conf,
                verbose=False,
            )[0]

            infer_s = time.perf_counter() - t0
            fps_now = 1.0 / max(infer_s, 1e-6)
            fps_smooth = (
                fps_now if fps_smooth == 0.0
                else 0.9 * fps_smooth + 0.1 * fps_now
            )

            annotated = result.plot()
            max_conf = 0.0
            count = 0
            if result.boxes is not None and len(result.boxes) > 0:
                count = len(result.boxes)
                max_conf = max(float(x) for x in result.boxes.conf)

            cv2.putText(
                annotated,
                f"detections: {count}  max conf: {max_conf:.2f}  infer FPS: {fps_smooth:.1f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            cv2.imshow("Shuttle YOLO Test", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
