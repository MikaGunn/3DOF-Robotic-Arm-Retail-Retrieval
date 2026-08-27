#!/bin/bash

SCRIPT="python3 /home/hamza/Logistics-Robot-with-Computer-Vision/testing/analyse_video.py"
VIDEOS="/home/hamza/Videos/supermarket_videos"
MODELS_DIR="/home/hamza/Logistics-Robot-with-Computer-Vision/computer_vision/models"

declare -A MODELS
MODELS["V3"]="$MODELS_DIR/V3weights.pt"
MODELS["V4"]="$MODELS_DIR/Model V4 - CPU object Detection Model.pt"
MODELS["V5"]="$MODELS_DIR/Model V5 - YOLOv11 no augs.pt"

declare -A VIDEOS_CLASSES
VIDEOS_CLASSES["apple1.MOV"]="Apple"
VIDEOS_CLASSES["apple2.MOV"]="Apple"
VIDEOS_CLASSES["banana1.MOV"]="Banana"
VIDEOS_CLASSES["banana2.MOV"]="Banana"
VIDEOS_CLASSES["orange_and_lemon.MOV"]="Orange"
VIDEOS_CLASSES["false_positive_test.MOV"]=""

for MODEL_NAME in "V3" "V4" "V5"; do
    export MODEL_PATH="${MODELS[$MODEL_NAME]}"
    echo "========================================"
    echo "Testing model: $MODEL_NAME"
    echo "========================================"

    for VIDEO in "${!VIDEOS_CLASSES[@]}"; do
        CLASS="${VIDEOS_CLASSES[$VIDEO]}"
        OUTPUT_DIR="$VIDEOS/${MODEL_NAME}"
        mkdir -p "$OUTPUT_DIR"

        echo ""
        echo "--- $VIDEO ($CLASS) ---"
        $SCRIPT "$VIDEOS/$VIDEO" "$CLASS" "$OUTPUT_DIR"
    done
done

echo ""
echo "All tests complete."
