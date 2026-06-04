"""
Dataset Capture for Metashape 3D Reconstruction
================================================
Captures undistorted images + per-image brightness masks from the IP camera.
Designed for photographing a PVC pipe structure underwater.

Output:
    dataset/
      images/         undistorted JPEGs  (photo_001.jpg ...)
      masks/          binary PNGs, same stem (photo_001.png)
      calibration.xml Metashape pre-calibration file

Usage:
    python3 capture_dataset.py [--rtsp-url URL] [--brightness-threshold 160] [--output dataset]

Controls (live window):
    p      pause / resume auto-capture
    m      toggle mask overlay (green = masked region / PVC)
    + / -  raise / lower brightness threshold
    q/ESC  quit and write calibration.xml

Auto-captures every 0.5 seconds by default.
"""

import argparse
import os
import sys
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np

# ── Calibration (from module_tests/CameraCalibration.py) ─────────────────────
CAMERA_MATRIX = np.array([
    [1.21914953e+03, 0.00000000e+00, 6.50249065e+02],
    [0.00000000e+00, 1.22765810e+03, 3.92846556e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float64)

DIST_COEFFS = np.array(
    [-0.51138646, 0.63217622, 0.02835033, 0.03921761, -1.17236853],
    dtype=np.float64
)

CALIB_W, CALIB_H = 1462, 792

RTSP_URLS = [
    "rtsp://192.168.2.64:554/stream1",
    "rtsp://192.168.2.64:554/live",
    "rtsp://192.168.2.64:554/cam/realmonitor?channel=1&subtype=0",
    "rtsp://192.168.2.64:554/Streaming/Channels/101",
    "rtsp://192.168.2.64:554/h264Preview_01_main",
    "rtsp://admin:admin@192.168.2.64:554/stream1",
]


# ── Camera helpers ────────────────────────────────────────────────────────────

def try_connect(url, timeout=5):
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, timeout * 1000)
    cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, timeout * 1000)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"  Connected: {frame.shape[1]}x{frame.shape[0]}  {url}")
            return cap
    cap.release()
    return None


def build_undistort_maps(cam_matrix, dist_coeffs, width, height):
    # alpha=0: crop to only valid (undistorted) pixels — no black borders,
    # no distorted edges leaking into the saved images.
    new_cam, roi = cv2.getOptimalNewCameraMatrix(
        cam_matrix, dist_coeffs, (width, height), 0, (width, height)
    )
    map_x, map_y = cv2.initUndistortRectifyMap(
        cam_matrix, dist_coeffs, None, new_cam, (width, height), cv2.CV_32FC1
    )
    return map_x, map_y, new_cam, roi


# ── Masking ───────────────────────────────────────────────────────────────────

def make_mask(frame, threshold):
    """
    Brightness-based mask for underwater PVC pipe (white/bright) against
    dark water background.  Returns a binary image: white = pipe, black = water.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]
    v_blur = cv2.GaussianBlur(v, (7, 7), 0)
    _, mask = cv2.threshold(v_blur, threshold, 255, cv2.THRESH_BINARY)
    # Close to fill gaps inside the pipe cross-section
    k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)
    # Open to remove small bright specks (reflections, bubbles)
    k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)
    return mask


# ── Calibration XML ───────────────────────────────────────────────────────────

def write_calibration_xml(path, new_cam, width, height):
    """
    Metashape pre-calibration XML.  Distortion coefficients are 0 because
    images are already undistorted; fx/fy/cx/cy come from the optimal new
    camera matrix so Metashape knows the correct projection.
    """
    root = ET.Element("calibration")
    root.set("type", "frame")
    root.set("class", "adjusted")

    res = ET.SubElement(root, "resolution")
    res.set("width", str(width))
    res.set("height", str(height))

    for tag, val in [
        ("fx", new_cam[0, 0]), ("fy", new_cam[1, 1]),
        ("cx", new_cam[0, 2]), ("cy", new_cam[1, 2]),
        ("k1", 0), ("k2", 0), ("k3", 0), ("p1", 0), ("p2", 0),
    ]:
        ET.SubElement(root, tag).text = f"{val:.6f}"

    tree = ET.ElementTree(root)
    tree.write(path, xml_declaration=True, encoding="utf-8")
    print(f"Calibration XML: {path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Metashape dataset capture")
    parser.add_argument("--rtsp-url", default=None)
    parser.add_argument("--brightness-threshold", type=int, default=160,
                        help="Initial brightness threshold 0-255 (default 160)")
    parser.add_argument("--output", default="dataset",
                        help="Output directory (default: dataset)")
    args = parser.parse_args()

    print("=" * 60)
    print("  DATASET CAPTURE — Metashape 3D Reconstruction")
    print("=" * 60)
    print()
    print("  *** IMPORTANT: PVC pipe has almost no surface texture. ***")
    print("  Apply colored adhesive tape strips (3-4 colors, random")
    print("  spacing) along the PVC frame BEFORE shooting. This is")
    print("  the most impactful step for reconstruction quality.")
    print()

    # Connect
    cap = None
    if args.rtsp_url:
        print(f"Connecting to {args.rtsp_url} ...")
        cap = try_connect(args.rtsp_url)
    else:
        print("Auto-detecting camera on 192.168.2.64 ...")
        for url in RTSP_URLS:
            cap = try_connect(url)
            if cap:
                break

    if cap is None:
        print("\nERROR: Could not connect to camera.")
        print("Use --rtsp-url to specify the stream URL manually.")
        sys.exit(1)

    ret, frame = cap.read()
    if not ret:
        print("ERROR: Could not read first frame.")
        sys.exit(1)

    h, w = frame.shape[:2]

    # Scale camera matrix if stream resolution differs from calibration
    cam_matrix = CAMERA_MATRIX.copy()
    if w != CALIB_W or h != CALIB_H:
        sx, sy = w / CALIB_W, h / CALIB_H
        cam_matrix[0, 0] *= sx;  cam_matrix[0, 2] *= sx
        cam_matrix[1, 1] *= sy;  cam_matrix[1, 2] *= sy
        print(f"Calibration scaled {CALIB_W}x{CALIB_H} → {w}x{h}")

    print("Building undistortion maps ...")
    map_x, map_y, new_cam, roi = build_undistort_maps(cam_matrix, DIST_COEFFS, w, h)
    rx, ry, rw, rh = roi  # valid-pixel crop rectangle

    # Output dirs
    images_dir = os.path.join(args.output, "images")
    masks_dir  = os.path.join(args.output, "masks")
    os.makedirs(images_dir, exist_ok=True)
    os.makedirs(masks_dir,  exist_ok=True)

    threshold = args.brightness_threshold
    show_mask = False
    show_compare = False
    paused = False
    count = 0
    last_capture = 0.0
    interval = 0.5

    print(f"\nOutput directory: {args.output}/")
    print("Auto-capturing every 0.5s.")
    print("Controls: p=pause/resume  o=compare original/undistorted  m=mask  +/-=threshold  q=quit\n")
    print("TIP: Press 'o' first to verify the fisheye is being corrected before capturing.\n")

    cv2.namedWindow("Dataset Capture", cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Stream lost — reconnecting ...")
            time.sleep(1)
            continue

        # Undistort and crop to valid-pixel region (no black borders)
        remapped = cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR)
        undistorted = remapped[ry:ry+rh, rx:rx+rw]
        mask = make_mask(undistorted, threshold)

        # Build display
        if show_compare:
            # Side-by-side: original (resized) | undistorted
            orig_resized = cv2.resize(frame, (undistorted.shape[1], undistorted.shape[0]))
            display = np.hstack([orig_resized, undistorted])
            cv2.putText(display, "ORIGINAL (distorted)",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(display, "UNDISTORTED (saved)",
                        (undistorted.shape[1] + 10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        elif show_mask:
            overlay = np.zeros_like(undistorted)
            overlay[:, :, 1] = mask
            display = cv2.addWeighted(undistorted, 0.65, overlay, 0.55, 0)
        else:
            display = undistorted.copy()

        status = "PAUSED" if paused else f"REC  next in {max(0, interval - (time.time() - last_capture)):.1f}s"
        cv2.putText(display, f"Photos: {count}   Threshold: {threshold}   {status}",
                    (10, display.shape[0] - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
        cv2.putText(display, "p=pause  o=compare  m=mask  +/-=threshold  q=quit",
                    (10, display.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

        cv2.imshow("Dataset Capture", display)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q'), 27):
            break
        elif key in (ord('p'), ord('P')):
            paused = not paused
            print(f"  {'Paused' if paused else 'Resumed'}")
        elif key in (ord('o'), ord('O')):
            show_compare = not show_compare
            show_mask = False
        elif key in (ord('+'), ord('=')):
            threshold = min(255, threshold + 5)
            print(f"  Threshold → {threshold}")
        elif key in (ord('-'), ord('_')):
            threshold = max(0, threshold - 5)
            print(f"  Threshold → {threshold}")
        elif key in (ord('m'), ord('M')):
            show_mask = not show_mask
            show_compare = False

        # Auto-capture every 0.5 seconds
        if not paused and (time.time() - last_capture) >= interval:
            last_capture = time.time()
            count += 1
            stem = f"photo_{count:03d}"
            img_path  = os.path.join(images_dir, f"{stem}.jpg")
            mask_path = os.path.join(masks_dir,  f"{stem}.png")
            cv2.imwrite(img_path,  undistorted, [cv2.IMWRITE_JPEG_QUALITY, 95])
            cv2.imwrite(mask_path, mask)
            coverage = mask.mean() / 255 * 100
            print(f"  Saved {stem}  (mask coverage {coverage:.1f}%)")

    cap.release()
    cv2.destroyAllWindows()

    if count == 0:
        print("No photos captured.")
        return

    xml_path = os.path.join(args.output, "calibration.xml")
    write_calibration_xml(xml_path, new_cam, rw, rh)

    print(f"\n{count} photos saved to {args.output}/")
    print()
    print("Metashape import steps:")
    print(f"  1. Add Photos → {images_dir}/")
    print(f"  2. Tools > Camera Calibration > Import {xml_path}")
    print(f"       Type = Precalibrated,  check 'Fix Calibration'")
    print(f"  3. Import Masks > From File > {masks_dir}/")
    print(f"  4. Align Photos → Apply masks to: Key points")
    print(f"  5. Resize bounding region tightly around the PVC structure")
    print(f"  6. Build Dense Cloud → Apply masks")
    print(f"  7. Build Mesh from dense cloud")


if __name__ == "__main__":
    main()
