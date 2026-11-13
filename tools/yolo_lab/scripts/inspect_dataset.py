#!/usr/bin/env python3
"""Run the migrated original dataset validator and persist machine-readable QA.

The original validator remains authoritative for malformed labels, exact
duplicate images, split emptiness, and render preview creation.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


def inspect(root, samples=24):
    root = Path(root).resolve()
    if not (root / "data.yaml").is_file():
        raise ValueError("Missing dataset data.yaml; generate a dataset first")
    command = [sys.executable, str(Path(__file__).with_name("validate_generated_dataset.py")),
               "--dataset", str(root), "--samples", str(samples)]
    result = subprocess.run(command, capture_output=True, text=True)
    report = {
        "updated_wall": time.time(),
        "dataset": str(root),
        "passed": result.returncode == 0,
        "returncode": result.returncode,
        "stdout_tail": result.stdout[-12000:],
        "stderr_tail": result.stderr[-5000:],
        "previews": sorted(p.name for p in (root / "preview").glob("*")
                           if p.suffix.lower() in (".jpg", ".jpeg", ".png")),
        "splits": {},
        "labels": 0,
        "negative_images": 0,
    }
    for split in ("train", "val", "test"):
        images = list((root / "images" / split).glob("*.jpg"))
        labels = list((root / "labels" / split).glob("*.txt"))
        negatives = sum(not p.read_text().strip() for p in labels)
        boxes = sum(len([x for x in p.read_text().splitlines() if x.strip()])
                    for p in labels)
        report["splits"][split] = {
            "images": len(images), "labels": len(labels),
            "negatives": negatives, "boxes": boxes}
        report["labels"] += boxes
        report["negative_images"] += negatives
    outfile = root / "inspection.json"
    tmp = root / ".inspection.tmp"
    tmp.write_text(json.dumps(report, indent=2) + "\n")
    tmp.replace(outfile)
    print(json.dumps({
        "passed": report["passed"], "previews": len(report["previews"]),
        "splits": report["splits"], "dataset": report["dataset"]}), flush=True)
    if not report["passed"]:
        print(report["stdout_tail"], file=sys.stderr, flush=True)
        raise SystemExit(1)
    return report


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--dataset",required=True)
    parser.add_argument("--samples",type=int,default=24)
    args=parser.parse_args()
    inspect(args.dataset,args.samples)


if __name__=="__main__":
    main()
