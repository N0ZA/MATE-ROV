"""
Real-time RTSP Fisheye Undistortion
===================================
Connects to a fisheye camera via RTSP and applies lens distortion
correction using pre-calibrated parameters.

Camera IP: 192.168.2.64
Calibration source: checkerboard calibration (13x9 inner corners)

Usage:
    python rtsp_undistort.py [--rtsp-url <url>] [--width <w>] [--height <h>]

Requirements:
    pip install opencv-python numpy
"""

import cv2
import numpy as np
import argparse
import sys
import time

# ── Calibration Parameters (from checkerboard calibration) ───────────────────
CAMERA_MATRIX = np.array([
    [1.21914953e+03, 0.00000000e+00, 6.50249065e+02],
    [0.00000000e+00, 1.22765810e+03, 3.92846556e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float64)

DIST_COEFFS = np.array([
    -0.51138646, 0.63217622, 0.02835033, 0.03921761, -1.17236853
], dtype=np.float64)

# ── RTSP URL patterns (common for IP cameras) ───────────────────────────────
RTSP_URLS = [
    "rtsp://192.168.2.64:554/stream1",
    "rtsp://192.168.2.64:554/live",
    "rtsp://192.168.2.64:554/cam/realmonitor?channel=1&subtype=0",
    "rtsp://192.168.2.64:554/Streaming/Channels/101",
    "rtsp://192.168.2.64:554/h264Preview_01_main",
    "rtsp://admin:admin@192.168.2.64:554/stream1",
]


def build_undistort_maps(cam_matrix, dist_coeffs, width, height):
    """Pre-compute the undistortion maps for fast remapping."""
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        cam_matrix, dist_coeffs, (width, height), 0, (width, height)
    )
    map_x, map_y = cv2.initUndistortRectifyMap(
        cam_matrix, dist_coeffs, None, new_camera_matrix,
        (width, height), cv2.CV_32FC1
    )
    return map_x, map_y, roi


def try_connect(url, timeout=5):
    """Attempt to connect to an RTSP stream."""
    print(f"  Trying: {url}")
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, timeout * 1000)
    cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, timeout * 1000)

    if cap.isOpened():
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"  ✓ Connected! Stream resolution: {frame.shape[1]}x{frame.shape[0]}")
            return cap
    cap.release()
    return None


def main():
    parser = argparse.ArgumentParser(description="Real-time RTSP fisheye undistortion")
    parser.add_argument("--rtsp-url", type=str, default=None,
                        help="Full RTSP URL (overrides auto-detection)")
    parser.add_argument("--width", type=int, default=None,
                        help="Resize width for display (default: stream native)")
    parser.add_argument("--height", type=int, default=None,
                        help="Resize height for display (default: stream native)")
    parser.add_argument("--show-original", action="store_true",
                        help="Show original and undistorted side by side")
    parser.add_argument("--record", type=str, default=None,
                        help="Record undistorted output to file (e.g. output.mp4)")
    args = parser.parse_args()

    # ── Connect to RTSP stream ───────────────────────────────────────────────
    cap = None
    if args.rtsp_url:
        print(f"Connecting to: {args.rtsp_url}")
        cap = try_connect(args.rtsp_url)
        if cap is None:
            print(f"ERROR: Could not connect to {args.rtsp_url}")
            sys.exit(1)
    else:
        print("Auto-detecting RTSP stream on 192.168.2.64 ...")
        for url in RTSP_URLS:
            cap = try_connect(url)
            if cap is not None:
                break
        if cap is None:
            print("\nERROR: Could not connect to any RTSP URL.")
            print("Please specify the URL manually with --rtsp-url")
            print("Example: python rtsp_undistort.py --rtsp-url rtsp://192.168.2.64:554/stream1")
            sys.exit(1)

    # ── Read first frame to get dimensions ───────────────────────────────────
    ret, frame = cap.read()
    if not ret:
        print("ERROR: Could not read from stream.")
        sys.exit(1)

    h, w = frame.shape[:2]
    print(f"\nStream resolution: {w}x{h}")

    # Scale camera matrix if stream resolution differs from calibration (1462x792)
    calib_w, calib_h = 1462, 792
    cam_matrix_scaled = CAMERA_MATRIX.copy()
    if w != calib_w or h != calib_h:
        sx = w / calib_w
        sy = h / calib_h
        cam_matrix_scaled[0, 0] *= sx  # fx
        cam_matrix_scaled[1, 1] *= sy  # fy
        cam_matrix_scaled[0, 2] *= sx  # cx
        cam_matrix_scaled[1, 2] *= sy  # cy
        print(f"Scaled calibration from {calib_w}x{calib_h} → {w}x{h}")

    # ── Pre-compute undistortion maps ────────────────────────────────────────
    print("Building undistortion maps...")
    map_x, map_y, roi = build_undistort_maps(cam_matrix_scaled, DIST_COEFFS, w, h)
    print("Maps ready. Starting live view...\n")

    # ── Optional video writer ────────────────────────────────────────────────
    writer = None
    if args.record:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = cap.get(cv2.CAP_PROP_FPS) or 25
        writer = cv2.VideoWriter(args.record, fourcc, fps, (w, h))
        print(f"Recording to: {args.record}")

    # ── Display settings ─────────────────────────────────────────────────────
    display_w = args.width or w
    display_h = args.height or h
    if args.show_original:
        display_w = display_w // 2  # each panel is half width

    window_name = "Fisheye Undistortion (press Q to quit)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    fps_counter = 0
    fps_time = time.time()
    fps_display = 0.0

    # ── Main loop ────────────────────────────────────────────────────────────
    print("Controls:")
    print("  Q / ESC  - Quit")
    print("  S        - Save screenshot")
    print("  O        - Toggle original/undistorted side-by-side")
    print()

    show_side_by_side = args.show_original

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Stream ended or connection lost. Reconnecting...")
            cap.release()
            time.sleep(2)
            cap = cv2.VideoCapture(args.rtsp_url or RTSP_URLS[0], cv2.CAP_FFMPEG)
            continue

        # Undistort using pre-computed maps (fast)
        undistorted = cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR)

        # FPS calculation
        fps_counter += 1
        elapsed = time.time() - fps_time
        if elapsed >= 1.0:
            fps_display = fps_counter / elapsed
            fps_counter = 0
            fps_time = time.time()

        # Build display frame
        if show_side_by_side:
            display = np.hstack([
                cv2.resize(frame, (display_w, display_h)),
                cv2.resize(undistorted, (display_w, display_h))
            ])
            cv2.putText(display, "Original", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(display, "Undistorted", (display_w + 10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            display = cv2.resize(undistorted, (display_w * (2 if args.show_original else 1), display_h))

        # FPS overlay
        cv2.putText(display, f"FPS: {fps_display:.1f}", (display.shape[1] - 150, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow(window_name, display)

        if writer:
            writer.write(undistorted)

        # Key handling
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):  # Q or ESC
            break
        elif key in (ord('s'), ord('S')):
            filename = f"screenshot_{int(time.time())}.jpg"
            cv2.imwrite(filename, undistorted)
            print(f"Screenshot saved: {filename}")
        elif key in (ord('o'), ord('O')):
            show_side_by_side = not show_side_by_side

    # ── Cleanup ──────────────────────────────────────────────────────────────
    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()