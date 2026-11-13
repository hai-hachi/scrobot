#!/usr/bin/env python3

import argparse
import csv
import hashlib
import random
import shutil
from pathlib import Path

import cv2


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args():
    p = argparse.ArgumentParser(
        description="Validate a generated YOLO dataset and render bbox previews."
    )
    p.add_argument(
        "--dataset",
        default="dataset/gazebo_scrobot_simple",
        help="Dataset root containing images/{train,val,test} and labels/{train,val,test}.",
    )
    p.add_argument(
        "--samples",
        type=int,
        default=12,
        help="Total preview images. Samples are distributed across train/val/test.",
    )
    p.add_argument("--seed", type=int, default=42)
    return p.parse_args()


def list_images(path):
    if not path.exists():
        return []
    return sorted(
        p for p in path.iterdir()
        if p.is_file() and p.suffix.lower() in IMAGE_EXTS
    )


def read_yolo_label(path):
    boxes = []
    errors = []

    if not path.exists():
        return boxes, [f"missing label: {path}"]

    text = path.read_text().strip()
    if not text:
        return boxes, errors  # Valid negative frame.

    for line_no, line in enumerate(text.splitlines(), 1):
        parts = line.split()
        if len(parts) != 5:
            errors.append(f"{path}:{line_no}: expected 5 fields, got {len(parts)}")
            continue

        try:
            cls = int(parts[0])
            xc, yc, w, h = map(float, parts[1:])
        except ValueError:
            errors.append(f"{path}:{line_no}: non-numeric YOLO row")
            continue

        if cls != 0:
            errors.append(f"{path}:{line_no}: unexpected class id {cls}")

        for name, value in (("xc", xc), ("yc", yc), ("w", w), ("h", h)):
            if not (0.0 <= value <= 1.0):
                errors.append(
                    f"{path}:{line_no}: {name}={value:.6f} outside [0,1]"
                )

        if w <= 0.0 or h <= 0.0:
            errors.append(f"{path}:{line_no}: non-positive width/height")

        boxes.append((cls, xc, yc, w, h))

    return boxes, errors


def draw_preview(image_path, label_path, out_path):
    image = cv2.imread(str(image_path))
    if image is None:
        return False, f"could not read image: {image_path}"

    boxes, errors = read_yolo_label(label_path)
    h_img, w_img = image.shape[:2]

    for _, xc, yc, w, h in boxes:
        x0 = int(round((xc - w / 2.0) * w_img))
        y0 = int(round((yc - h / 2.0) * h_img))
        x1 = int(round((xc + w / 2.0) * w_img))
        y1 = int(round((yc + h / 2.0) * h_img))

        x0 = max(0, min(w_img - 1, x0))
        x1 = max(0, min(w_img - 1, x1))
        y0 = max(0, min(h_img - 1, y0))
        y1 = max(0, min(h_img - 1, y1))

        cv2.rectangle(image, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.putText(
            image,
            "Shuttlecock",
            (x0, max(16, y0 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out_path), image):
        return False, f"could not write preview: {out_path}"

    return True, errors



def file_sha1(path):
    h = hashlib.sha1()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def load_metadata(root):
    path = root / "metadata.csv"
    if not path.exists():
        return {}
    rows = {}
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            frame = row.get("frame", "")
            if frame:
                rows[frame] = row
    return rows


def main():
    args = parse_args()
    repo_root = Path(__file__).resolve().parent.parent
    root = Path(args.dataset).expanduser()
    if not root.is_absolute():
        root = repo_root / root
    root = root.resolve()
    rng = random.Random(args.seed)

    if not root.exists():
        raise SystemExit(f"Dataset not found: {root}")

    metadata = load_metadata(root)

    print(f"Dataset: {root}")
    print()

    total_images = 0
    total_labels = 0
    total_boxes = 0
    total_negatives = 0
    all_pairs = []
    pairs_by_split = {"train": [], "val": [], "test": []}
    errors = []
    image_hashes = {}

    split_counts = {}

    for split in ("train", "val", "test"):
        img_dir = root / "images" / split
        lbl_dir = root / "labels" / split

        images = list_images(img_dir)
        labels = sorted(lbl_dir.glob("*.txt")) if lbl_dir.exists() else []

        image_stems = {p.stem for p in images}
        label_stems = {p.stem for p in labels}

        missing_labels = sorted(image_stems - label_stems)
        orphan_labels = sorted(label_stems - image_stems)

        boxes_in_split = 0
        negatives = 0

        for image_path in images:
            label_path = lbl_dir / f"{image_path.stem}.txt"
            boxes, label_errors = read_yolo_label(label_path)
            errors.extend(label_errors)
            boxes_in_split += len(boxes)
            if len(boxes) == 0:
                negatives += 1
            pair = (split, image_path, label_path)
            all_pairs.append(pair)
            pairs_by_split[split].append(pair)

            digest = file_sha1(image_path)
            image_hashes.setdefault(digest, []).append(image_path)

        split_counts[split] = len(images)
        total_images += len(images)
        total_labels += len(labels)
        total_boxes += boxes_in_split
        total_negatives += negatives

        print(
            f"{split:5s}: images={len(images):4d}  labels={len(labels):4d}  "
            f"boxes={boxes_in_split:5d}  negatives={negatives:4d}"
        )

        if missing_labels:
            errors.append(
                f"{split}: {len(missing_labels)} images missing label files"
            )
        if orphan_labels:
            errors.append(
                f"{split}: {len(orphan_labels)} labels without matching images"
            )

    print()
    if total_images:
        print(
            "Split ratio: "
            f"train={100.0*split_counts['train']/total_images:.1f}%  "
            f"val={100.0*split_counts['val']/total_images:.1f}%  "
            f"test={100.0*split_counts['test']/total_images:.1f}%"
        )
        print(
            f"Total: images={total_images} labels={total_labels} "
            f"boxes={total_boxes} negatives={total_negatives}"
        )
    else:
        errors.append("dataset contains no images")

    if split_counts.get("train", 0) == 0:
        errors.append("train split is empty")
    if split_counts.get("val", 0) == 0:
        errors.append("val split is empty")

    duplicate_groups = [
        paths for paths in image_hashes.values()
        if len(paths) > 1
    ]
    if duplicate_groups:
        print()
        print(f"Duplicate image hashes: {len(duplicate_groups)} group(s)")
        stale_pose_groups = 0
        for paths in duplicate_groups[:10]:
            print("  " + " == ".join(str(p.relative_to(root)) for p in paths))

            poses = []
            for p in paths:
                row = metadata.get(p.stem)
                if row:
                    pose = (
                        row.get("camera_x", "?"),
                        row.get("camera_y", "?"),
                        row.get("camera_yaw_rad", "?"),
                    )
                    poses.append((p.stem, pose))

            if poses:
                unique_poses = {pose for _, pose in poses}
                if len(unique_poses) > 1:
                    stale_pose_groups += 1
                    print("    SAME PIXELS, DIFFERENT COMMANDED POSES:")
                    for stem, pose in poses:
                        print(
                            f"      {stem}: x={pose[0]} y={pose[1]} yaw={pose[2]}"
                        )

        if stale_pose_groups:
            errors.append(
                f"{stale_pose_groups} duplicate-image group(s) have different "
                "camera poses; this indicates stale/reused rendered RGB frames"
            )
        else:
            errors.append(
                f"{len(duplicate_groups)} exact duplicate image group(s) found"
            )

    print()
    preview_root = root / "preview"
    if preview_root.exists():
        shutil.rmtree(preview_root)
    preview_root.mkdir(parents=True, exist_ok=True)

    requested = max(0, args.samples)
    sampled = []

    # Stratify previews across all non-empty splits so validation cannot
    # accidentally show only train images.
    nonempty_splits = [
        split for split in ("train", "val", "test")
        if pairs_by_split[split]
    ]
    if requested and nonempty_splits:
        base = requested // len(nonempty_splits)
        remainder = requested % len(nonempty_splits)

        for idx, split in enumerate(nonempty_splits):
            want = base + (1 if idx < remainder else 0)
            want = min(want, len(pairs_by_split[split]))
            if want:
                sampled.extend(rng.sample(pairs_by_split[split], want))

        # If a small split could not satisfy its quota, fill remaining slots
        # from the unused global pool.
        if len(sampled) < requested:
            used = {pair[1] for pair in sampled}
            remaining = [
                pair for pair in all_pairs
                if pair[1] not in used
            ]
            extra = min(requested - len(sampled), len(remaining))
            if extra:
                sampled.extend(rng.sample(remaining, extra))

    sample_count = len(sampled)

    preview_errors = []
    for index, (split, image_path, label_path) in enumerate(sampled, 1):
        out_path = preview_root / f"{index:02d}_{split}_{image_path.name}"
        print(
            f"preview {index:02d}: {split}  "
            f"{image_path.name}  <-  {label_path.name}"
        )
        ok, detail = draw_preview(image_path, label_path, out_path)
        if not ok:
            preview_errors.append(str(detail))
        elif detail:
            preview_errors.extend(detail)

    errors.extend(preview_errors)

    print(f"Preview images: {preview_root}")
    print(f"Rendered samples: {sample_count}")

    print()
    if errors:
        print(f"CHECK RESULT: FAIL/WARN ({len(errors)} issue(s))")
        for item in errors[:30]:
            print(f"  - {item}")
        if len(errors) > 30:
            print(f"  ... plus {len(errors)-30} more")
        raise SystemExit(1)

    print("CHECK RESULT: STRUCTURAL PASS")
    print("Labels are structurally valid, image/label pairs match by stem, and train/val are non-empty.")
    print("This script cannot prove geometric synchronization from text files alone.")
    print("Open the freshly regenerated preview directory and visually confirm every green box covers the shuttle in that same image.")


if __name__ == "__main__":
    main()
