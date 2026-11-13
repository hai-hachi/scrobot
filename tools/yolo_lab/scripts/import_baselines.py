#!/usr/bin/env python3
"""Optional, verified migration of legacy model weights.

All training/evaluation SOURCE code is in this repository. The three
large binary checkpoints are downloaded on explicit request, pinned
to their immutable Git blob SHA-1, and never committed to scrobot.
"""
from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import tempfile
from urllib.request import urlopen

LAB = Path(__file__).resolve().parents[1]
MODELS = LAB / "models/pretrained"
SOURCE = "https://raw.githubusercontent.com/hai-hachi/yoloshuttle/main/"
BLOBS = {
    "best.pt": (6224810, "3384e99ea845d63d503f87e03547745f836d7b4e"),
    "yolo11n_shuttle.pt": (5477786, "a397f783cc636e513127893fd0688c6ecae60884"),
    "yolo11s-ball.pt": (19173075, "01325395e4da8317c0da40e6db7695297fdbbe73"),
}


def sha_git_blob(payload):
    return hashlib.sha1(b"blob " + str(len(payload)).encode() + b"\0" + payload).hexdigest()


def import_checkpoint(name, target_dir=MODELS):
    if name not in BLOBS:
        raise ValueError("Only the three pinned original YOLO checkpoints can be imported")
    size, sha = BLOBS[name]
    target_dir = Path(target_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    out = target_dir / name
    if out.exists() and out.stat().st_size == size and sha_git_blob(out.read_bytes()) == sha:
        return out
    # Explicit operator action. Never download model bytes during server start.
    with urlopen(SOURCE + name, timeout=60) as response:
        data = response.read(size + 1)
    if len(data) != size or sha_git_blob(data) != sha:
        raise ValueError("Downloaded checkpoint did not match the pinned Git blob SHA")
    tmp = None
    try:
        with tempfile.NamedTemporaryFile(dir=target_dir, delete=False) as f:
            tmp = Path(f.name)
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, out)
        tmp = None
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)
    return out


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("name", choices=list(BLOBS))
    args=parser.parse_args()
    print("Verified model imported:", import_checkpoint(args.name), flush=True)


if __name__ == "__main__":
    main()
