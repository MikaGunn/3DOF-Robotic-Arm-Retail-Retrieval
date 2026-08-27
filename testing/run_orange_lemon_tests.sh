#!/bin/bash

SCRIPT="python3 /home/hamza/Logistics-Robot-with-Computer-Vision/testing/analyse_video.py"
VIDEOS="/home/hamza/Videos/supermarket_videos"
MODELS_DIR="/home/hamza/Logistics-Robot-with-Computer-Vision/computer_vision/models"

declare -A MODELS
MODELS["V3"]="$MODELS_DIR/V3weights.pt"
MODELS["V5"]="$MODELS_DIR/Model V5 - YOLOv11 no augs.pt"

for MODEL_NAME in "V3" "V5"; do
    export MODEL_PATH="${MODELS[$MODEL_NAME]}"
    echo "========================================"
    echo "Testing model: $MODEL_NAME"
    echo "========================================"

    echo "--- lemon1.mov (Lemon) ---"
    $SCRIPT "$VIDEOS/lemon1.mov" Lemon "$VIDEOS/$MODEL_NAME"

    echo "--- orange1.mov (Orange) ---"
    $SCRIPT "$VIDEOS/orange1.mov" Orange "$VIDEOS/$MODEL_NAME"
done

echo ""
echo "Done."
