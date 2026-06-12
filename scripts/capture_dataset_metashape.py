import cv2
import time
import signal
from datetime import datetime
from pathlib import Path

RTSP_URL         = "rtsp://admin:admin@192.168.2.12:554/"
CAPTURE_INTERVAL = 0.2
DATASET_ROOT     = Path(__file__).resolve().parent.parent / "dataset"

running = True

def _stop(sig, frame):
    global running
    running = False

signal.signal(signal.SIGTERM, _stop)
signal.signal(signal.SIGINT,  _stop)

output_dir = DATASET_ROOT / f"metashape_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
output_dir.mkdir(parents=True, exist_ok=True)

cap = cv2.VideoCapture(RTSP_URL)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
count = 0

while running:
    ret, frame = cap.read()
    if not ret:
        cap.open(RTSP_URL)
        continue
    cv2.imwrite(str(output_dir / f"frame_{count:05d}.jpg"), frame,
                [cv2.IMWRITE_JPEG_QUALITY, 95])
    count += 1
    print(count, flush=True)
    time.sleep(CAPTURE_INTERVAL)

cap.release()
print(f"DONE:{count}:{output_dir}", flush=True)
