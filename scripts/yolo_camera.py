import cv2
import time
from pathlib import Path
from ultralytics import YOLO

RTSP_URL = "rtsp://admin:admin@192.168.2.14:554/"
MODEL_PATH = Path(__file__).resolve().parent.parent / "best.pt"
CONF_THRESHOLD = 0.4

model = YOLO(str(MODEL_PATH))

cap = cv2.VideoCapture(RTSP_URL)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print(f"Error: Could not open stream at {RTSP_URL}")
    exit(1)

print(f"Stream opened. Model: {MODEL_PATH.name}  |  Press 'q' to quit")

fps_time = time.time()
fps = 0.0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Stream read failed — retrying...")
        cap.open(RTSP_URL)
        continue

    results = model(frame, conf=CONF_THRESHOLD, verbose=False)[0]

    european_crab_count = 0
    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])
        label = f"{model.names[cls_id]} {conf:.2f}"

        if cls_id == 1 and conf > 0.75:
            european_crab_count += 1

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), (0, 255, 0), -1)
        cv2.putText(frame, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    now = time.time()
    fps = 0.9 * fps + 0.1 * (1.0 / max(now - fps_time, 1e-6))
    fps_time = now
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 255), 2)
    cv2.putText(frame, f"European Crabs (>75%): {european_crab_count}", (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)

    cv2.imshow("ROV YOLO Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
