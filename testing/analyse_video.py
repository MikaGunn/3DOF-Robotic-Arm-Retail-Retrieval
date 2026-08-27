import cv2
import time
import sys
import os
import matplotlib.pyplot as plt
from ultralytics import YOLO

CLASS_NAMES = {0: 'Apple', 1: 'Banana', 2: 'Lemon', 3: 'Orange'}
CONF_THRESHOLD = 0.482

model_path = os.environ.get('MODEL_PATH',
    '/home/hamza/Logistics-Robot-with-Computer-Vision/computer_vision/models/V3weights.pt')
model = YOLO(model_path)

video_path = sys.argv[1]
expected_class = sys.argv[2] if len(sys.argv) > 2 else None
output_dir = sys.argv[3] if len(sys.argv) > 3 else os.path.dirname(video_path)

cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    print(f"Error opening video: {video_path}")
    sys.exit(1)

confidences = []
proc_times = []
true_positive = 0
false_positive = 0
missed = 0
total_frames = 0

print(f"Processing: {video_path}")
print(f"Expected class: {expected_class}")
print("-" * 50)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    total_frames += 1
    start = time.time()
    results = model.predict(source=frame, conf=CONF_THRESHOLD, verbose=False)
    elapsed = time.time() - start
    proc_times.append(elapsed * 1000)

    boxes = results[0].boxes if results and results[0].boxes else []

    if boxes:
        best_conf = 0
        best_class = None
        for box in boxes:
            conf = float(box.conf[0].item())
            cls = int(box.cls[0].item())
            if conf > best_conf:
                best_conf = conf
                best_class = CLASS_NAMES.get(cls, '?')

        confidences.append(best_conf)

        if expected_class:
            if best_class == expected_class:
                true_positive += 1
            else:
                false_positive += 1
                print(f"Frame {total_frames}: False positive - detected {best_class} (conf={best_conf:.3f})")
    else:
        confidences.append(0)
        if expected_class:
            missed += 1

cap.release()

print(f"\nResults for {os.path.basename(video_path)}:")
print(f"Total frames: {total_frames}")
if expected_class:
    print(f"True positive: {true_positive} ({100*true_positive/total_frames:.1f}%)")
    print(f"False positive: {false_positive} ({100*false_positive/total_frames:.1f}%)")
    print(f"Missed: {missed} ({100*missed/total_frames:.1f}%)")
avg_conf = sum([c for c in confidences if c > 0]) / max(len([c for c in confidences if c > 0]), 1)
print(f"Average confidence (detected frames): {avg_conf:.3f}")
print(f"Average processing time: {sum(proc_times)/len(proc_times):.1f}ms")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

ax1.plot(confidences, color='blue', linewidth=0.8, label='Confidence')
ax1.axhline(y=CONF_THRESHOLD, color='red', linestyle='--', label=f'Threshold ({CONF_THRESHOLD})')
ax1.set_xlabel('Frame')
ax1.set_ylabel('Confidence')
ax1.set_title(f'Detection Confidence per Frame - {os.path.basename(video_path)}')
ax1.legend()
ax1.set_ylim(0, 1)

ax2.plot(proc_times, color='green', linewidth=0.8, label='Processing time')
ax2.axhline(y=sum(proc_times)/len(proc_times), color='red', linestyle='--',
            label=f'Average ({sum(proc_times)/len(proc_times):.1f}ms)')
ax2.set_xlabel('Frame')
ax2.set_ylabel('Time (ms)')
ax2.set_title(f'Processing Time per Frame - {os.path.basename(video_path)}')
ax2.legend()

plt.tight_layout()
output_name = os.path.basename(video_path).replace('.MOV', '').replace('.mov', '') + '_analysis.png'
output_path = os.path.join(output_dir, output_name)
plt.savefig(output_path, dpi=150)
print(f"Plot saved to: {output_path}")
plt.close()
