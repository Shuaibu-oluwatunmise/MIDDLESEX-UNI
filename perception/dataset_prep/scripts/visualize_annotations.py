"""
Visualize ALL YOLO annotations for FSOCO cone detection dataset
"""
import cv2
from pathlib import Path
from tqdm import tqdm

def plot_yolo_boxes(img_path, label_path, output_path, class_names):
    """Draw YOLO format bounding boxes on image"""
    # Read image
    img = cv2.imread(str(img_path))
    if img is None:
        print(f"Could not read image: {img_path}")
        return False
    
    h, w = img.shape[:2]
    
    # Read labels
    if not label_path.exists():
        print(f"No label file for {img_path}")
        return False
        
    with open(label_path, 'r') as f:
        lines = f.readlines()
    
    # Color map: Blue, Yellow, Orange
    colors = {
        0: (255, 0, 0),    # Blue cone -> Blue color
        1: (0, 255, 255),  # Yellow cone -> Yellow color  
        2: (0, 165, 255),  # Orange cone -> Orange color
    }
    
    cone_counts = {0: 0, 1: 0, 2: 0}
    
    # Draw each box
    for line in lines:
        parts = line.strip().split()
        if len(parts) < 5:
            continue
            
        class_id = int(parts[0])
        x_center = float(parts[1]) * w
        y_center = float(parts[2]) * h
        box_w = float(parts[3]) * w
        box_h = float(parts[4]) * h
        
        # Convert to corner coordinates
        x1 = int(x_center - box_w/2)
        y1 = int(y_center - box_h/2)
        x2 = int(x_center + box_w/2)
        y2 = int(y_center + box_h/2)
        
        # Get color for this class
        color = colors.get(class_id, (255, 255, 255))
        
        # Draw box
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        
        # Draw label with background
        label = class_names[class_id]
        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        
        # Draw filled rectangle for label background
        cv2.rectangle(img, (x1, y1 - label_size[1] - 10), 
                     (x1 + label_size[0] + 5, y1), color, -1)
        
        # Draw label text
        cv2.putText(img, label, (x1 + 2, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.6, (0, 0, 0), 2)
        
        cone_counts[class_id] += 1
    
    # Add summary text at top
    total_cones = sum(cone_counts.values())
    summary = f"Total: {total_cones} | Blue: {cone_counts[0]} | Yellow: {cone_counts[1]} | Orange: {cone_counts[2]}"
    cv2.putText(img, summary, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                0.8, (255, 255, 255), 2)
    cv2.putText(img, summary, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                0.8, (0, 0, 0), 1)
    
    # Save
    cv2.imwrite(str(output_path), img)
    return True

# ============================================
# CONFIGURATION
# ============================================

print("=" * 70)
print("Visualizing FSOCO Cone Detection Annotations (ALL IMAGES)")
print("=" * 70)

data_folder = Path('../YOLO_DATA_FSOCO')
class_names = ['blue_cone', 'yellow_cone', 'orange_cone']
output_folder = Path('../visualizations/YOLO_DATA_FSOCO')

if not data_folder.exists():
    print(f"❌ YOLO_DATA_FSOCO not found!")
    exit(1)

total_visualized = 0

# ============================================
# VISUALIZE ALL IMAGES IN ALL SPLITS
# ============================================

for split in ['train', 'val', 'test']:
    img_dir = data_folder / split / 'images'
    lbl_dir = data_folder / split / 'labels'
    output_dir = output_folder / split
    
    if not img_dir.exists():
        print(f"\n⚠️  {split}/ not found, skipping...")
        continue
    
    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Get ALL images
    all_images = list(img_dir.glob('*.*'))
    
    print(f"\n📸 Processing {split}/ ({len(all_images)} images)...")
    
    success_count = 0
    for img_file in tqdm(all_images, desc=f"  {split}"):
        label_file = lbl_dir / f"{img_file.stem}.txt"
        output_file = output_dir / img_file.name
        
        if plot_yolo_boxes(img_file, label_file, output_file, class_names):
            success_count += 1
    
    total_visualized += success_count
    print(f"   ✓ Saved {success_count} visualizations to {output_dir}")

# ============================================
# SUMMARY
# ============================================

print("\n" + "=" * 70)
print("✅ VISUALIZATION COMPLETE!")
print("=" * 70)
print(f"\n📁 Output: visualizations/YOLO_DATA_FSOCO/")
print(f"   📊 Total visualized: {total_visualized} images")
print(f"   🚂 Train: visualizations/YOLO_DATA_FSOCO/train/")
print(f"   ✅ Val: visualizations/YOLO_DATA_FSOCO/val/")
print(f"   🧪 Test: visualizations/YOLO_DATA_FSOCO/test/")
print("\n💡 Color coding:")
print(f"   🔵 Blue cones")
print(f"   🟡 Yellow cones")
print(f"   🟠 Orange cones")
print("=" * 70)