#!/usr/bin/env python3
"""
train_hardware_model.py

Trains a YOLO11n-seg model on the new 5-class hardware dataset (V2 --
without tiling, per the plan to keep single-shot inference for now).

Runs locally. Note: if torch on this machine reports the CUDA driver
mismatch warning seen earlier, training will silently fall back to CPU --
it'll still work, just considerably slower than a GPU run.

Output:
  checkpoints/   -- best.pt and last.pt, refreshed every time Ultralytics
                    saves a new one (last.pt every epoch, best.pt whenever
                    validation improves)
  log/           -- training_log.csv, one row appended per epoch with that
                    epoch's metrics
  runs/segment/hardware_finetune_v2/  -- Ultralytics' own full run output
                    (plots, confusion matrix, args.yaml) -- left in place,
                    checkpoints/ and log/ are convenience copies of the
                    two most-referenced pieces of it
"""

import csv
import os
import shutil
from ultralytics import YOLO

# ── Config -- edit these for your setup ─────────────────────────────────
DATA_YAML = "data.yaml"   # <-- point this at the V2 (non-tiled) export
BASE_MODEL = "yolo11n-seg.pt"                    # pretrained COCO checkpoint, auto-downloads
EPOCHS = 200
IMGSZ = 640
BATCH = 8            # drop to 4 if you hit memory pressure on CPU
PATIENCE = 30         # early stopping if val loss stalls this many epochs
PROJECT = "runs/segment"
RUN_NAME = "hardware_finetune_v2"

CHECKPOINTS_DIR = "checkpoints"
LOG_DIR = "log"
LOG_FILE = os.path.join(LOG_DIR, "training_log.csv")
# ─────────────────────────────────────────────────────────────────────────


def setup_dirs():
    os.makedirs(CHECKPOINTS_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)


def on_model_save(trainer):
    """Fires whenever Ultralytics writes last.pt (every epoch) or best.pt
    (whenever validation improves) -- mirror both into checkpoints/ so
    you've always got the current best/last there without digging through
    the full runs/ tree."""
    weights_dir = trainer.save_dir / "weights"
    for name in ("last.pt", "best.pt"):
        src = weights_dir / name
        if src.exists():
            shutil.copy2(src, os.path.join(CHECKPOINTS_DIR, name))


def on_fit_epoch_end(trainer):
    """Fires at the end of every epoch -- append that epoch's metrics as a
    row to log/training_log.csv. Column names come straight from
    trainer.metrics, so exact fields depend on your Ultralytics version --
    check the header row after epoch 1 if you want to confirm what's in
    there before writing anything that parses this file downstream."""
    metrics = trainer.metrics or {}
    row = {"epoch": trainer.epoch + 1, **metrics}

    write_header = not os.path.exists(LOG_FILE)
    with open(LOG_FILE, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def main():
    setup_dirs()

    model = YOLO(BASE_MODEL)
    model.add_callback("on_model_save", on_model_save)
    model.add_callback("on_fit_epoch_end", on_fit_epoch_end)

    model.train(
        data=DATA_YAML,
        epochs=EPOCHS,
        imgsz=IMGSZ,
        batch=BATCH,
        patience=PATIENCE,
        project=PROJECT,
        name=RUN_NAME,
        plots=True,
    )

    print("\nTraining complete.")
    print(f"Checkpoints: {CHECKPOINTS_DIR}/best.pt, {CHECKPOINTS_DIR}/last.pt")
    print(f"Per-epoch log: {LOG_FILE}")
    print(f"Full run output (plots, confusion matrix, args.yaml): {PROJECT}/{RUN_NAME}/")
    print("Copy checkpoints/best.pt to kuka_ros2_demo/data/ as a SEPARATE weights file "
          "(don't overwrite the current 16-class best.pt) until you've "
          "verified this one performs better.")


if __name__ == "__main__":
    main()
