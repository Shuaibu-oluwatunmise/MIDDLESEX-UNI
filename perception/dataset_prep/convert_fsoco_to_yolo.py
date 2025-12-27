"""
Convert FSOCO Supervisely format to YOLO format
Uses the existing human-labeled annotations from FSOCO
"""
import json
import shutil
from pathlib import Path
from tqdm import tqdm
import random

# Configuration
FSOCO_ROOT = Path("fsoco_bounding_boxes_train")
OUTPUT_DIR = Path("YOLO_DATA_FSOCO")

# Class mapping from FSOCO to YOLO
# Based on meta.json class definitions
CLASS_MAPPING = {
    "blue_cone": 0,
    "yellow_cone": 1,
    "orange_cone": 2,
    "large_orange_cone": 2,  # Merge large orange with regular orange
    "unknown_cone": -1  # Skip unknown cones
}

# Train/val/test split
TRAIN_SPLIT = 0.8
VAL_SPLIT = 0.1
TEST_SPLIT = 0.1

print("=" * 70)
print("Converting FSOCO to YOLO Format")
print("=" * 70)

# Create output structure
for split in ["train", "val", "test"]:
    (OUTPUT_DIR / split / "images").mkdir(parents=True, exist_ok=True)
    (OUTPUT_DIR / split / "labels").mkdir(parents=True, exist_ok=True)

# Find all team folders
team_folders = [f for f in FSOCO_ROOT.iterdir() if f.is_dir()]
print(f"\nFound {len(team_folders)} team folders")

# Collect all image-annotation pairs
all_data = []

print("\nScanning for images and annotations...")
for team_folder in tqdm(team_folders, desc="Teams"):
    img_folder = team_folder / "img"
    ann_folder = team_folder / "ann"
    
    if not img_folder.exists() or not ann_folder.exists():
        continue
    
    # Find all images with annotations
    for img_file in img_folder.glob("*.*"):
        ann_file = ann_folder / f"{img_file.name}.json"
        
        if ann_file.exists():
            all_data.append((img_file, ann_file, team_folder.name))

print(f"\n✅ Found {len(all_data)} images with annotations")

# Shuffle and split
random.seed(42)
random.shuffle(all_data)

train_end = int(len(all_data) * TRAIN_SPLIT)
val_end = train_end + int(len(all_data) * VAL_SPLIT)

splits = {
    "train": all_data[:train_end],
    "val": all_data[train_end:val_end],
    "test": all_data[val_end:]
}

print(f"\nSplit: {len(splits['train'])} train, {len(splits['val'])} val, {len(splits['test'])} test")

# Convert annotations
print("\n" + "=" * 70)
print("Converting to YOLO format...")
print("=" * 70)

converted_count = 0
skipped_count = 0
cone_stats = {"blue_cone": 0, "yellow_cone": 0, "orange_cone": 0}

for split_name, data_list in splits.items():
    print(f"\nProcessing {split_name} split...")
    
    for img_file, ann_file, team_name in tqdm(data_list, desc=f"  {split_name}"):
        try:
            # Read annotation JSON
            with open(ann_file, 'r') as f:
                ann_data = json.load(f)
            
            # Get image dimensions
            img_width = ann_data["size"]["width"]
            img_height = ann_data["size"]["height"]
            
            # Convert bounding boxes
            yolo_labels = []
            
            for obj in ann_data.get("objects", []):
                class_title = obj["classTitle"]
                
                # Skip if not in our mapping or is unknown
                if class_title not in CLASS_MAPPING:
                    continue
                
                class_id = CLASS_MAPPING[class_title]
                if class_id == -1:  # Skip unknown cones
                    continue
                
                # Get bounding box coordinates
                # FSOCO format: points.exterior = [[x1, y1], [x2, y2]]
                points = obj["points"]["exterior"]
                x1, y1 = points[0]
                x2, y2 = points[1]
                
                # Convert to YOLO format (normalized center_x, center_y, width, height)
                center_x = ((x1 + x2) / 2) / img_width
                center_y = ((y1 + y2) / 2) / img_height
                width = abs(x2 - x1) / img_width
                height = abs(y2 - y1) / img_height
                
                # Skip invalid boxes
                if width <= 0 or height <= 0:
                    continue
                
                yolo_labels.append(f"{class_id} {center_x:.6f} {center_y:.6f} {width:.6f} {height:.6f}")
                
                # Track stats
                for class_name, cid in CLASS_MAPPING.items():
                    if cid == class_id and cid != -1:
                        cone_stats[class_name] += 1
                        break
            
            # Skip images with no valid labels
            if not yolo_labels:
                skipped_count += 1
                continue
            
            # Create unique filename (team_originalname)
            new_img_name = f"{team_name}_{img_file.name}"
            new_label_name = f"{team_name}_{img_file.stem}.txt"
            
            # Copy image
            shutil.copy(img_file, OUTPUT_DIR / split_name / "images" / new_img_name)
            
            # Write YOLO label
            label_path = OUTPUT_DIR / split_name / "labels" / new_label_name
            with open(label_path, 'w') as f:
                f.write('\n'.join(yolo_labels))
            
            converted_count += 1
            
        except Exception as e:
            print(f"\n  ⚠️  Error: {img_file.name} - {e}")
            skipped_count += 1

# Create data.yaml
yaml_content = f"""# FSOCO Dataset - Human Labeled
path: {OUTPUT_DIR.absolute()}
train: train/images
val: val/images
test: test/images

nc: 3
names: ['blue_cone', 'yellow_cone', 'orange_cone']
"""

with open(OUTPUT_DIR / "data.yaml", "w") as f:
    f.write(yaml_content)

# Summary
print("\n" + "=" * 70)
print("✅ CONVERSION COMPLETE!")
print("=" * 70)

train_count = len(list((OUTPUT_DIR / "train" / "images").glob("*")))
val_count = len(list((OUTPUT_DIR / "val" / "images").glob("*")))
test_count = len(list((OUTPUT_DIR / "test" / "images").glob("*")))

print(f"\n📊 Dataset Statistics:")
print(f"   📁 Output: {OUTPUT_DIR.absolute()}")
print(f"   🏷️  Classes: blue_cone, yellow_cone, orange_cone")
print(f"   🚂 Train: {train_count} images")
print(f"   ✅ Val: {val_count} images")
print(f"   🧪 Test: {test_count} images")
print(f"   📝 Total: {train_count + val_count + test_count} images")
print(f"   ⏭️  Skipped: {skipped_count} images (no valid labels)")

print(f"\n🎯 Cone Statistics (total across all splits):")
print(f"   🔵 Blue cones: {cone_stats['blue_cone']:,}")
print(f"   🟡 Yellow cones: {cone_stats['yellow_cone']:,}")
print(f"   🟠 Orange cones: {cone_stats['orange_cone']:,}")
print(f"   📦 Total cones: {sum(cone_stats.values()):,}")

print(f"\n🎯 Ready for YOLOv8 training!")
print(f"   Train command: yolo train data={OUTPUT_DIR / 'data.yaml'} model=yolov8n.pt")
print("=" * 70)