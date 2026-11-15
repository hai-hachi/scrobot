#!/usr/bin/env python3
"""Entry point shared with other SCROBOT developer labs."""
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from planning.web_server import main

if __name__ == "__main__":
    main()
