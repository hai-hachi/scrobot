"""Strip terminal control bytes when showing subprocess logs in a browser.

The raw *.log files remain untouched for troubleshooting.
"""
from __future__ import annotations

import re

# ANSI CSI, OSC (including terminal hyperlinks), single ESC, and controls.
ANSI = re.compile(
    r"\x1b\[[0-?]*[ -/]*[@-~]"
    r"|\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)"
    r"|\x1b[@-_]"
)
CONTROLS = re.compile(r"[\x00-\x08\x0b\x0c\x0e-\x1f\x7f]")


def clean(text):
    return CONTROLS.sub("", ANSI.sub("", text))


def tail(path, max_bytes=3500):
    from pathlib import Path
    path = Path(path)
    if not path.is_file():
        return ""
    with path.open("rb") as stream:
        stream.seek(max(0, stream.seek(0, 2) - max_bytes))
        data = stream.read()
    return clean(data.decode("utf-8", errors="replace"))
