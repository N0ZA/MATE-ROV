#!/usr/bin/env python3
"""
Reads MJPEG multipart stream from stdin, applies fisheye undistortion
(LEFT ARM CAMERA, 192.168.2.12), and writes corrected MJPEG to stdout.
Spawn as a pipe between GStreamer and the Node.js server.
"""
import sys
import cv2
import numpy as np

K = np.array([[631.69336823, 0., 395.85630862],
              [0., 631.06251341, 251.80559769],
              [0., 0., 1.]], dtype=np.float64)
# Fisheye model: 4 k-coefficients, shaped (4,1)
D = np.array([[-9.83155346e-02], [3.74382797e-01],
              [-1.61267810e+00], [1.78189287e+00]], dtype=np.float64)

map1 = None
map2 = None

def ensure_maps(h, w):
    global map1, map2
    if map1 is not None:
        return
    Knew = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (w, h), np.eye(3), balance=0.0)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), Knew, (w, h), cv2.CV_16SC2)

buf = bytearray()
sin  = sys.stdin.buffer
sout = sys.stdout.buffer

while True:
    chunk = sin.read(65536)
    if not chunk:
        break
    buf += chunk

    while True:
        soi = buf.find(b'\xff\xd8')
        if soi == -1:
            if len(buf) > 3:
                buf = bytearray(buf[-3:])
            break
        eoi = buf.find(b'\xff\xd9', soi + 2)
        if eoi == -1:
            if soi > 0:
                buf = bytearray(buf[soi:])
            break

        jpeg = bytes(buf[soi:eoi + 2])
        buf  = bytearray(buf[eoi + 2:])

        img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue

        h, w = img.shape[:2]
        ensure_maps(h, w)

        out = cv2.remap(img, map1, map2, cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

        ok, enc = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            continue

        sout.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + enc.tobytes())
        sout.flush()
