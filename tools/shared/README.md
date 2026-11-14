# Shared lab infrastructure

- `lab_configs.py` — canonical lab YAML registry, validation-aware
  atomic read/save and fallback for old saved paths. YAMLs are under
  `tools/configs/labs/`.
- `log_text.py` — strip terminal control bytes from browser log tails,
  leaving original log files unchanged.

These modules are imported by individual labs. Compatibility import files
`tools/lab_configs.py` and `tools/log_text.py` remain so older scripts
do not need an immediate rewrite.
