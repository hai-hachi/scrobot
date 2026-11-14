"""Compatibility import. Canonical lab YAML registry: tools/shared/lab_configs.py.

Use shared.lab_configs in new code; existing lab entrypoints keep working.
"""
from shared.lab_configs import (ROOT, CONFIG_DIR, LEGACY_CONFIG_DIR, FILES,
                                path_for, read, save, matches,
                                lighting_for_consumer)
