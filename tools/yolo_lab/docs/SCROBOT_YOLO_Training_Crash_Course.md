# Repository Layout\n\nBefore the training concepts, know where everything lives:\n\n\`\`\`text\nyoloshuttle/\n├── scripts/     # all executable training / validation / test tools\n├── docs/        # notes like this crash course\n├── dataset/     # generated local datasets\n├── datasets/    # downloaded external datasets\n├── runs/        # raw Ultralytics training experiments\n├── artifacts/   # stable copies of trained best models\n└── *.pt         # tracked baseline / comparison weights\n\`\`\`\n\n- **scripts/**: run code from here, e.g. \`python scripts/train_finetune.py ...\`.\n- **docs/**: documentation only.\n- **dataset/**: current Gazebo and local datasets; ignored by Git.\n- **datasets/**: downloaded third-party datasets; ignored by Git.\n- **runs/**: full Ultralytics output including plots, CSV files, and best/last weights.\n- **artifacts/**: clean copies of the best trained models. \`artifacts/latest_model.txt\` points to the newest completed model.\n\n---\n\n# YOLO Training Crash Course for SCROBOT Shuttlecock Detection

## 1. What YOLO training does

YOLO learns from an image plus the correct bounding box for each shuttlecock. It predicts boxes, compares them with the labels, calculates loss, and updates the model weights.

```text
image
  ↓
YOLO prediction
  ↓
compare with ground truth
  ↓
calculate loss
  ↓
backpropagation
  ↓
update weights
  ↓
repeat
```

For this project there is one class:

```text
0 = Shuttlecock
```

A YOLO label row is:

```text
0 x_center y_center width height
```

The four box values are normalized to 0–1.

## 2. Train / Validation / Test

**Train** updates model weights. **Validation** checks generalization during training without updating weights. **Test** is used after training to evaluate the final model on unseen images.

A 70% / 15% / 15% split is suitable for the generated dataset.

## 3. Fine-tuning

The current workflow starts from `best.pt`, so this is fine-tuning rather than training from scratch:

```text
existing shuttlecock detector
          +
new SCROBOT Gazebo images
          ↓
fine-tuned detector
```

This is useful because the baseline already contains real-world shuttlecock features.

## 4. Epochs

One epoch means one full pass through the training set. About 15 epochs is a sensible first synthetic fine-tune. Too many synthetic-only epochs can improve Gazebo performance while hurting real-camera performance.

## 5. Batch size

`batch=8` means the GPU processes 8 images before one optimizer update. Higher batch sizes use more VRAM and can improve throughput. For the RTX 2000 Ada laptop GPU, 8 is safe; try 16 later if memory allows it.

## 6. Image size

Current training uses `imgsz=640`. This is a good baseline, but shuttlecocks are small objects. If distant-shuttle recall is poor, later compare `imgsz=960` or larger if GPU memory allows.

## 7. Learning rate

Current value:

```text
lr0 = 0.0001
```

This is intentionally conservative because the model is already trained and we only want to adapt it rather than overwrite useful features.

## 8. Losses

YOLO reports values such as:

```text
box_loss
cls_loss
dfl_loss
```

Lower is generally better, but training loss alone does not prove the model is good. Validation metrics matter more.

## 9. Precision

Precision answers: of all detections YOLO called shuttlecocks, how many were actually shuttlecocks?

```text
Precision = TP / (TP + FP)
```

High precision means fewer false positives such as court lines, reflections, or net features.

## 10. Recall

Recall answers: of all real shuttlecocks present, how many did YOLO find?

```text
Recall = TP / (TP + FN)
```

For a collection robot, recall is especially important because missed shuttlecocks may remain on the court.

## 11. IoU

IoU measures overlap between predicted and ground-truth boxes:

```text
IoU = intersection area / union area
```

Perfect overlap is 1.0.

## 12. mAP@50

mAP@50 evaluates detections using IoU ≥ 0.50. For SCROBOT this is particularly relevant because the robot mainly needs a sufficiently good box for heading and depth estimation.

## 13. mAP@50–95

This averages mAP from IoU 0.50 to 0.95 and is much stricter. Small shuttlecocks are hard to localize perfectly, so this will naturally be lower than mAP@50.

## 14. Confidence threshold

Lower confidence thresholds improve recall but may add false positives. Higher thresholds improve precision but may miss more shuttlecocks. Tune confidence on real-camera data, not only synthetic validation.

## 15. Augmentation

YOLO may use scale, translation, HSV changes, mosaic, flipping, and cropping. The training script uses `close_mosaic=10`, so mosaic is disabled near the end of training for refinement on more natural-looking images.

## 16. Overfitting

Typical overfitting looks like training loss continuing to fall while validation performance stops improving or worsens. For this project, also watch for synthetic-domain overfitting: excellent Gazebo metrics but worse D435i performance.

## 17. Patience / Early stopping

`patience=8` means training can stop if validation performance has not improved for about 8 epochs.

## 18. best.pt vs last.pt

Ultralytics normally produces:

```text
weights/
├── best.pt
└── last.pt
```

Use `best.pt` for evaluation and deployment. The reorganized repo also copies it to:

```text
artifacts/models/<run_name>.pt
```

# Target Metrics for SCROBOT

The final target should not be based only on Gazebo metrics. Use three levels: training health, synthetic test performance, and real D435i performance.

## A. Minimum acceptable synthetic test target

Aim for at least:

```text
Precision   ≥ 0.90
Recall      ≥ 0.90
mAP@50      ≥ 0.90
mAP@50-95   ≥ 0.50
```

If Gazebo metrics are significantly below these values, there is likely still room to improve labels, image diversity, image size, training settings, or model capacity.

## B. Good synthetic target

A strong synthetic result would be approximately:

```text
Precision   ≥ 0.95
Recall      ≥ 0.95
mAP@50      ≥ 0.95
mAP@50-95   ≥ 0.60
```

Because this is a single-class synthetic dataset with native Gazebo boxes, these are reasonable stretch targets. However, very high Gazebo scores do not guarantee real-world performance.

## C. Real-world target for SCROBOT

For real D435i green-court images, aim for:

```text
Recall      ≥ 0.90
Precision   ≥ 0.90
mAP@50      ≥ 0.85
```

A very good deployment target would be:

```text
Recall      ≥ 0.95
Precision   ≥ 0.92
mAP@50      ≥ 0.90
```

For SCROBOT, recall should be given slightly more importance than extremely tight boxes.

## D. Recommended priority order

```text
1. Real-world recall
2. Real-world precision
3. Detection stability across consecutive frames
4. mAP@50
5. mAP@50-95
```

A model with strong recall and adequate box localization can be more useful to the robot than a model with very tight boxes but frequent misses.

## E. Compare against the current baseline

Previous reported baseline:

```text
Precision   = 0.8684
Recall      = 0.7549
mAP@50      = 0.7740
mAP@50-95   = 0.2581
```

A meaningful improvement on real validation data would initially be:

```text
Precision   > 0.90
Recall      > 0.85
mAP@50      > 0.85
mAP@50-95   > 0.35
```

A stronger long-term target is:

```text
Precision   ≥ 0.92
Recall      ≥ 0.90
mAP@50      ≥ 0.90
mAP@50-95   ≥ 0.45
```

# Practical Decision Rule

Do not replace the baseline model just because synthetic metrics are better.

```text
baseline best.pt
        VS
new fine-tuned model
        ↓
same real green-court test images
        ↓
compare:
- missed shuttlecocks
- false positives
- confidence
- frame-to-frame stability
- detection range
```

Only replace the deployed model when the new model is clearly better on the real robot camera.

# Current Recommended Training Commands

Conservative baseline:

```bash
python scripts/train_finetune.py \
  --model best.pt \
  --data dataset/gazebo_scrobot_simple/data.yaml \
  --device 0 \
  --epochs 15 \
  --batch 8 \
  --imgsz 640 \
  --optimizer AdamW \
  --lr0 0.0001 \
  --patience 8 \
  --workers 4 \
  --name gazebo_simple_v1
```

More aggressive small-object fine-tune:

```bash
python scripts/train_finetune.py \
  --model artifacts/models/gazebo_simple_v1.pt \
  --data dataset/gazebo_scrobot_simple/data.yaml \
  --device 0 \
  --epochs 40 \
  --batch 8 \
  --imgsz 960 \
  --optimizer AdamW \
  --lr0 0.001 \
  --cos-lr \
  --patience 12 \
  --workers 4 \
  --name gazebo_simple_v2
```

Then evaluate:

```bash
python scripts/validate_model.py latest \
  --data dataset/gazebo_scrobot_simple/data.yaml \
  --device 0 \
  --imgsz 640 \
  --batch 16 \
  --split test
```

Compare with baseline:

```bash
python scripts/validate_model.py best.pt \
  --data dataset/gazebo_scrobot_simple/data.yaml \
  --device 0 \
  --imgsz 640 \
  --batch 16 \
  --split test
```

Final acceptance should come from real D435i green-court testing.
