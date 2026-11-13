#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

DATA_PATH="${1:-dataset/gazebo_scrobot_simple/data.yaml}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

echo "============================================================"
echo "YOLOSHUTTLE Ubuntu/CUDA smoke test"
echo "repo     : ${REPO_ROOT}"
echo "dataset  : ${DATA_PATH}"
echo "python   : ${PYTHON_BIN}"
echo "============================================================"

if ! command -v nvidia-smi >/dev/null 2>&1; then
  echo "ERROR: nvidia-smi is not available."
  exit 2
fi

echo
echo "[1/5] NVIDIA driver/GPU"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader

if [ ! -d ".venv" ]; then
  echo
  echo "[2/5] Creating virtual environment"
  "${PYTHON_BIN}" -m venv .venv
fi

source .venv/bin/activate

echo
echo "[3/5] Installing/updating Python dependencies"
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

echo
echo "[4/5] Checking PyTorch CUDA"
python - <<'PY'
import sys
import torch

print("Python       :", sys.version.split()[0])
print("PyTorch      :", torch.__version__)
print("torch CUDA   :", torch.version.cuda)
print("CUDA usable  :", torch.cuda.is_available())

if not torch.cuda.is_available():
    raise SystemExit("CUDA is not available inside PyTorch.")

print("GPU          :", torch.cuda.get_device_name(0))
free, total = torch.cuda.mem_get_info(0)
print(f"VRAM free    : {free / 1024**3:.2f} GiB")
print(f"VRAM total   : {total / 1024**3:.2f} GiB")
PY

if [ ! -f "${DATA_PATH}" ]; then
  echo
  echo "CUDA/PyTorch check PASSED."
  echo "Dataset not found at: ${DATA_PATH}"
  echo "Generate it with scrobot_debug/yolo_simple_dataset.launch.py first."
  exit 0
fi

echo
echo "Dataset split counts:"
train_imgs=0
val_imgs=0
for split in train val test; do
  img_dir="$(dirname "${DATA_PATH}")/images/${split}"
  label_dir="$(dirname "${DATA_PATH}")/labels/${split}"
  imgs=0
  labels=0
  [ -d "${img_dir}" ] && imgs="$(find "${img_dir}" -type f | wc -l)"
  [ -d "${label_dir}" ] && labels="$(find "${label_dir}" -type f | wc -l)"
  echo "  ${split}: images=${imgs} labels=${labels}"
  [ "${split}" = "train" ] && train_imgs="${imgs}"
  [ "${split}" = "val" ] && val_imgs="${imgs}"
done

if [ "${train_imgs}" -eq 0 ] || [ "${val_imgs}" -eq 0 ]; then
  echo "ERROR: training needs non-empty train and val splits."
  exit 3
fi

echo
echo "[5/5] One-epoch CUDA fine-tuning smoke"
python scripts/train_finetune.py \
  --model best.pt \
  --data "${DATA_PATH}" \
  --device 0 \
  --epochs 1 \
  --batch 4 \
  --imgsz 640 \
  --lr0 0.0001 \
  --patience 2 \
  --workers 2 \
  --name ubuntu_cuda_smoke

echo
echo "============================================================"
echo "CUDA TRAINING SMOKE PASSED"
echo "run weights : runs/detect/ubuntu_cuda_smoke/weights/best.pt"
echo "stable copy : artifacts/models/ubuntu_cuda_smoke.pt"
echo "============================================================"
