import argparse
import cv2
import time
import signal
from datetime import datetime
from pathlib import Path

DEFAULT_RTSP = 'rtsp://admin:admin@192.168.2.12:554/live/0/SUB'
CAPTURE_INTERVAL = 0.2
DATASET_ROOT     = Path(__file__).resolve().parent.parent / "dataset"

parser = argparse.ArgumentParser()
parser.add_argument('--rtsp-url', default=DEFAULT_RTSP)
parser.add_argument('--zoom', type=float, default=1.0)
args = parser.parse_args()

RTSP_URL = args.rtsp_url
zoom     = max(1.0, args.zoom)

running = True

def _stop(sig, frame):
    global running
    running = False

signal.signal(signal.SIGTERM, _stop)
signal.signal(signal.SIGINT,  _stop)


def apply_zoom(frame, z):
    """Center-crop to replicate the CSS scale(z) zoom shown in the UI."""
    if z <= 1.0:
        return frame
    h, w = frame.shape[:2]
    ch, cw = int(h / z), int(w / z)
    y0 = (h - ch) // 2
    x0 = (w - cw) // 2
    return cv2.resize(frame[y0:y0 + ch, x0:x0 + cw], (w, h), interpolation=cv2.INTER_LINEAR)


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
    frame = apply_zoom(frame, zoom)
    cv2.imwrite(str(output_dir / f"frame_{count:05d}.jpg"), frame,
                [cv2.IMWRITE_JPEG_QUALITY, 95])
    count += 1
    print(count, flush=True)
    time.sleep(CAPTURE_INTERVAL)

cap.release()
print(f"DONE:{count}:{output_dir}", flush=True)
