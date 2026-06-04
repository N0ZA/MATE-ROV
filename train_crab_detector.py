import os
import shutil
import random
import torch
from pathlib import Path
from ultralytics import YOLO

# --- CONFIGURATION ---
DATASET_DIR   = 'crab_dataset_labeled'
# yolov8n.pt  → fastest, lowest VRAM  (recommended for ROV real-time use)
# yolov8s.pt  → slightly more accurate, still fast
# yolo26l.pt  → high accuracy, slower (use only for offline testing)
BASE_MODEL    = 'yolov8n.pt'
EPOCHS        = 50
IMG_SIZE      = 320            # 320 is 4× faster than 640 with minimal accuracy loss
BATCH_SIZE    = 16             # nano model fits larger batches
VAL_SPLIT     = 0.15
RANDOM_SEED   = 42
PROJECT_NAME  = 'crab_training'
RUN_NAME      = 'crab_detector_v1'

DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"Using device: {DEVICE}")


def build_val_split(dataset_dir: str, val_ratio: float, seed: int):
    """
    Moves a random subset of train images/labels into val/ folders.
    Safe to re-run: skips if val/ already has files.
    """
    img_train = Path(dataset_dir) / 'images' / 'train'
    lbl_train = Path(dataset_dir) / 'labels' / 'train'
    img_val   = Path(dataset_dir) / 'images' / 'val'
    lbl_val   = Path(dataset_dir) / 'labels' / 'val'

    img_val.mkdir(parents=True, exist_ok=True)
    lbl_val.mkdir(parents=True, exist_ok=True)

    # Already split — nothing to do
    if any(img_val.iterdir()):
        print(f"Val split already exists ({len(list(img_val.iterdir()))} images). Skipping split.")
        return

    all_images = sorted(img_train.glob('*.png')) + sorted(img_train.glob('*.jpg'))
    random.seed(seed)
    random.shuffle(all_images)

    n_val = max(1, int(len(all_images) * val_ratio))
    val_images = all_images[:n_val]

    for img_path in val_images:
        lbl_path = lbl_train / (img_path.stem + '.txt')
        shutil.move(str(img_path), str(img_val / img_path.name))
        if lbl_path.exists():
            shutil.move(str(lbl_path), str(lbl_val / lbl_path.name))

    print(f"Split: {len(all_images) - n_val} train / {n_val} val images")


def fix_data_yaml(dataset_dir: str):
    """Rewrites data.yaml with the correct absolute path and real val split."""
    abs_path = str(Path(dataset_dir).resolve())
    yaml_path = Path(dataset_dir) / 'data.yaml'
    yaml_content = f"""path: {abs_path}
train: images/train
val: images/val
nc: 3
names:
  0: Jonah Crab
  1: European Crab
  2: Rock Crab
"""
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"data.yaml updated: {yaml_path}")
    return str(yaml_path)


def train(yaml_path: str):
    model = YOLO(BASE_MODEL)

    results = model.train(
        data=yaml_path,
        epochs=EPOCHS,
        imgsz=IMG_SIZE,
        batch=BATCH_SIZE,
        device=DEVICE,
        project=PROJECT_NAME,
        name=RUN_NAME,
        exist_ok=True,
        patience=15,       # early-stop if no improvement for 15 epochs
        save=True,
        plots=True,
    )

    best_weights = Path(PROJECT_NAME) / RUN_NAME / 'weights' / 'best.pt'
    print(f"\nTraining complete.")
    print(f"Best model saved to: {best_weights}")
    return results


if __name__ == '__main__':
    build_val_split(DATASET_DIR, VAL_SPLIT, RANDOM_SEED)
    yaml_path = fix_data_yaml(DATASET_DIR)
    train(yaml_path)
