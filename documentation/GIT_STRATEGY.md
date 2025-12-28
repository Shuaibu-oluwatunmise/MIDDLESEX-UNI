# Git Tracking Strategy

## ✅ What WILL Be Tracked on GitHub

### Training Results & Metrics
```
perception/dataset_prep/runs/
├── train/
│   └── cone_detector_s/
│       ├── results.csv          ✅ Track (performance metrics)
│       ├── results.png          ✅ Track (training curves)
│       ├── confusion_matrix.png ✅ Track (model analysis)
│       ├── BoxF1_curve.png      ✅ Track (precision/recall)
│       ├── args.yaml            ✅ Track (training config)
│       ├── train_batch*.jpg     ✅ Track (sample visualizations)
│       └── val_batch*.jpg       ✅ Track (validation samples)
```

### Model Weights
```
models/
├── yolov8n/weights/
│   ├── best.pt                  ✅ Track (final model)
│   └── last.pt                  ✅ Track (last checkpoint)
├── yolov8s/weights/
│   ├── best.pt                  ✅ Track
│   └── last.pt                  ✅ Track
└── yolov8m/weights/
    ├── best.pt                  ✅ Track
    └── last.pt                  ✅ Track
```

### Code & Documentation
```
perception/
├── dataset_prep/scripts/        ✅ Track (all .py files)
├── python_pipeline/             ✅ Track (all .py files)
├── README.md                    ✅ Track
└── requirements.txt             ✅ Track

documentation/                   ✅ Track (all .md files)
README.md                        ✅ Track
```

---

## ❌ What WON'T Be Tracked (Too Large)

### Datasets
```
perception/dataset_prep/
├── fsoco_bounding_boxes_train/  ❌ Ignore (24GB - download separately)
├── YOLO_DATA_FSOCO/             ❌ Ignore (processed dataset)
├── YOLO_DATA/                   ❌ Ignore (alternative format)
├── annotations_*/               ❌ Ignore (intermediate files)
└── visualizations/              ❌ Ignore (can regenerate)
```

### Archives
```
archive/                         ❌ Ignore (old training runs)
```

### Temporary Files
```
__pycache__/                     ❌ Ignore (Python cache)
*.log                            ❌ Ignore (logs)
.vscode/                         ❌ Ignore (IDE settings)
```

---

## 📊 Why This Strategy?

### ✅ Track Training Results Because:
1. **Documentation** - Shows model performance over time
2. **Collaboration** - Team can see what works
3. **Reproducibility** - Training configs preserved
4. **Small Size** - CSVs and PNGs are tiny (<1MB total)

### ✅ Track Model Weights Because:
1. **Deployment Ready** - Anyone can download and use
2. **Version Control** - Track model improvements
3. **Reasonable Size** - ~22MB per model (acceptable for GitHub)

### ❌ Don't Track Datasets Because:
1. **Too Large** - 24GB exceeds GitHub limits
2. **Publicly Available** - Can download from FSOCO website
3. **Redundant** - Everyone has their own copy

---

## 🚀 GitHub Workflow

### Initial Push
```bash
cd MIDDLESEX-UNI
git init
git add .
git commit -m "Initial commit: Perception module with YOLOv8s (74.7% mAP50)"
git branch -M main
git remote add origin <your-repo-url>
git push -u origin main
```

### After Training New Model
```bash
# Training results automatically saved to runs/
# Model weights saved to models/

git add perception/dataset_prep/runs/
git add models/
git commit -m "Training run: YOLOv8m - improved mAP50 to 76%"
git push
```

### What Teammates Will Get
When they clone:
```bash
git clone <repo-url>
cd MIDDLESEX-UNI

# They get:
✅ All code
✅ All training results (metrics, charts)
✅ All model weights (ready to use)
✅ All documentation

# They DON'T get (need to download separately):
❌ FSOCO dataset (they download from fsoco.cs.uni-freiburg.de)
❌ Your archived runs (not needed)
```

---

## 💡 Pro Tips

### If a Model is Too Large
If a model exceeds 100MB, use Git LFS:
```bash
git lfs install
git lfs track "*.pt"
git add .gitattributes
```

### Sharing Datasets
Create a `DATASET_SETUP.md`:
```markdown
# Dataset Setup

1. Download FSOCO dataset (24GB):
   https://fsoco.cs.uni-freiburg.de/

2. Extract to:
   perception/dataset_prep/fsoco_bounding_boxes_train/

3. Run conversion:
   cd perception/dataset_prep/scripts
   python convert_fsoco_to_yolo.py
```

### Repository Size
Your repo will be:
- **Code + Docs:** ~1MB
- **Training Results:** ~5MB (all runs)
- **Model Weights:** ~50MB (all models)
- **Total:** ~56MB ✅ Perfect for GitHub!

---

## 📁 What Your GitHub Repo Will Look Like

```
MIDDLESEX-UNI/                   (Public repo)
├── README.md                    ← Project overview
├── .gitignore                   ← This config
│
├── perception/
│   ├── README.md                ← Module guide
│   ├── dataset_prep/
│   │   ├── scripts/             ← Training code
│   │   ├── runs/                ← Training results ✨
│   │   └── requirements.txt
│   └── python_pipeline/         ← Inference code
│
├── models/                      ← Trained weights ✨
│   ├── yolov8n/
│   ├── yolov8s/                 ← Your best model
│   └── yolov8m/
│
├── planning/                    ← Future work
├── control/
├── simulation/
├── testing/
│
└── documentation/               ← Study guides
    ├── IMPLEMENTATION_ROADMAP.md
    ├── POST_PERCEPTION_PIPELINE.md
    └── ...
```

**Perfect for showcasing to sponsors, recruiters, and competition judges!** 🏆
