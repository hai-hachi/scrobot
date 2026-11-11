#!/usr/bin/env python3
import sys
import yaml
from monitor import run_monitor

if __name__ == '__main__':
    # argv: original config, run directory, immutable run config snapshot
    with open(sys.argv[3]) as f:
        config=yaml.safe_load(f)
    run_monitor(config,sys.argv[2])
