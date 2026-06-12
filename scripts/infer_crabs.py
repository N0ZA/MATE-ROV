#!/usr/bin/env python3
"""
Runs YOLO inference on a captured frame to detect European crabs.
Usage: python3 infer_crabs.py <input_image> <output_image>
Prints JSON to stdout: {"count": N, "class": "ClassName"}
Model is loaded fresh each run and exits after inference.
"""
import sys
import os
import json

KEYWORDS = ('european', 'pagurus', 'maenas', 'carcinus')

def find_european_indices(names):
    matches = [idx for idx, name in names.items()
               if any(kw in name.lower() for kw in KEYWORDS)]
    return matches if matches else list(names.keys())

def run(input_path, output_path):
    from ultralytics import YOLO
    import cv2

    model_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'best.pt'
    )
    model = YOLO(model_path)

    eu_indices = find_european_indices(model.names)
    display_name = model.names.get(eu_indices[0], 'European Crab') if eu_indices else 'European Crab'

    results = model(input_path, verbose=False)[0]
    img = cv2.imread(input_path)
    if img is None:
        print(json.dumps({'error': f'could not read {input_path}'}))
        sys.exit(1)

    count = 0
    for box in results.boxes:
        cls_id = int(box.cls[0])
        if cls_id not in eu_indices:
            continue
        count += 1
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])

        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 230, 100), 2)
        label = f'{display_name} {conf:.2f}'
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(img, (x1, y1 - th - 8), (x1 + tw + 4, y1), (0, 230, 100), -1)
        cv2.putText(img, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)

    # Count banner
    banner = f'European Crabs: {count}'
    bw = len(banner) * 13 + 16
    cv2.rectangle(img, (0, 0), (bw, 44), (0, 0, 0), -1)
    cv2.putText(img, banner, (8, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 230, 100), 2, cv2.LINE_AA)

    cv2.imwrite(output_path, img)
    print(json.dumps({'count': count, 'class': display_name}))

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(json.dumps({'error': 'usage: infer_crabs.py <input> <output>'}))
        sys.exit(1)
    run(sys.argv[1], sys.argv[2])
