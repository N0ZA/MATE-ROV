"""
Real-time crab detection from the ROV camera stream.

Runs as a separate process so the video pipeline is never blocked.
Detection results (bounding boxes) are served as JSON at http://localhost:5050/detections
The frontend can poll this endpoint and overlay boxes on the camera image with a <canvas>.

Usage:
    python detect_crabs.py
    python detect_crabs.py --model crab_training/crab_detector_v1/weights/best.pt
"""

import argparse
import json
import time
import threading
import urllib.request

import cv2
import torch
from http.server import BaseHTTPRequestHandler, HTTPServer
from ultralytics import YOLO

# --- CONFIGURATION ---
MODEL_PATH    = 'crab_training/crab_detector_v1/weights/best.pt'
CAMERA_URL    = 'http://localhost:3000/cam1'   # existing MJPEG stream
INFER_EVERY   = 3        # only run YOLO on every Nth frame (skip the rest)
INFER_SIZE    = 320      # inference resolution — must match training IMG_SIZE
CONF          = 0.45     # confidence threshold for live detections
RESULTS_PORT  = 5050     # JSON results served here

DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

# Shared state between detection thread and HTTP server
_lock       = threading.Lock()
_detections = []          # list of {label, conf, x1, y1, x2, y2, img_w, img_h}
_fps        = 0.0


# ---------------------------------------------------------------------------
# MJPEG frame reader
# ---------------------------------------------------------------------------

def iter_mjpeg_frames(url: str):
    """Generator that yields raw BGR frames from an MJPEG HTTP stream."""
    stream = urllib.request.urlopen(url, timeout=5)
    buf = b''
    while True:
        chunk = stream.read(4096)
        if not chunk:
            break
        buf += chunk
        start = buf.find(b'\xff\xd8')
        end   = buf.find(b'\xff\xd9')
        if start != -1 and end != -1 and end > start:
            jpg = buf[start:end + 2]
            buf = buf[end + 2:]
            frame = cv2.imdecode(
                __import__('numpy').frombuffer(jpg, dtype='uint8'),
                cv2.IMREAD_COLOR
            )
            if frame is not None:
                yield frame


# ---------------------------------------------------------------------------
# Detection loop (runs in background thread)
# ---------------------------------------------------------------------------

def detection_loop(model_path: str):
    global _detections, _fps

    print(f"Loading model: {model_path}")
    model = YOLO(model_path)
    model.to(DEVICE)

    frame_idx  = 0
    t_prev     = time.time()
    infer_count = 0

    print(f"Connecting to camera: {CAMERA_URL}")
    while True:
        try:
            for frame in iter_mjpeg_frames(CAMERA_URL):
                frame_idx += 1

                if frame_idx % INFER_EVERY != 0:
                    continue

                h, w = frame.shape[:2]
                results = model.predict(
                    frame,
                    imgsz=INFER_SIZE,
                    conf=CONF,
                    device=DEVICE,
                    verbose=False,
                )

                boxes = []
                for box in results[0].boxes:
                    cls_id = int(box.cls[0])
                    label  = model.names[cls_id]
                    conf   = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    boxes.append({
                        'label': label,
                        'conf':  round(conf, 3),
                        'x1': x1, 'y1': y1,
                        'x2': x2, 'y2': y2,
                        'img_w': w, 'img_h': h,
                    })

                infer_count += 1
                now  = time.time()
                fps  = infer_count / (now - t_prev) if now > t_prev else 0.0

                with _lock:
                    _detections = boxes
                    _fps        = round(fps, 1)

        except Exception as e:
            print(f"Stream error: {e} — retrying in 2s")
            time.sleep(2)


# ---------------------------------------------------------------------------
# Lightweight JSON HTTP server
# ---------------------------------------------------------------------------

class DetectionHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/detections':
            with _lock:
                payload = json.dumps({'fps': _fps, 'detections': _detections})
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(payload.encode())
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, *_):
        pass  # silence per-request logs


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=MODEL_PATH)
    args = parser.parse_args()

    t = threading.Thread(target=detection_loop, args=(args.model,), daemon=True)
    t.start()

    server = HTTPServer(('0.0.0.0', RESULTS_PORT), DetectionHandler)
    print(f"Detection results at http://localhost:{RESULTS_PORT}/detections")
    print(f"Device: {DEVICE} | Infer every {INFER_EVERY} frames | imgsz={INFER_SIZE}")
    server.serve_forever()
