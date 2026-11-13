# YOLO Lab scripts — ownership

These are child programs of `tools/yolo_lab/workflow.py` and are not
standalone production robot nodes. Launch the complete Lab with
`bash tools/yolo_lab/start.sh`.

| Responsibility | Script(s) |
|---|---|
| Triggered Gazebo dataset collection | `dataset_capture.py` |
| Structural label QA / previews | `inspect_dataset.py`, `validate_generated_dataset.py` |
| Fine-tune/checkpoint export | `train_finetune.py`, `plot_metrics.py` |
| Evaluation and measured Ultralytics plots | `validate_model.py` |
| Live ROS RGB inference | `live_detection.py` |
| Original checkpoint import / external dataset helper | `import_baselines.py`, `download_original_dataset.py` |
| Independent CLI diagnosis and CUDA smoke check | `test_shuttle.py`, `ubuntu_cuda_smoke.sh` |

Support modules in the **YOLO Lab root** own their feature:
`dataset_world.py`, `distribution.py`, `world.py`, `settle.py`,
`yolo_settings.py`, `preview.py`, `server.py` and `workflow.py`.

Lab-only ROS/Gazebo assets remain under `assets/` and `models/`.
Original user-renamed `.pt` weights belong in `models/pretrains/` and
are ignored by Git and protected from UI rename/delete.
