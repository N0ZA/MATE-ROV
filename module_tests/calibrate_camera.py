"""
ROV Camera Calibration
======================
Camera : rtsp://admin:Admin123@192.168.2.12:554/live/0/SUB
Board  : 13x9 inner corners  (14x10 squares — 7 black on long side, 5 on short side)

CONTROLS
--------
  SPACE  — capture frame (green dots = board detected)
  C      — calibrate now (needs 15+ frames)
  U      — toggle undistortion preview
  S      — save screenshot
  Q/ESC  — quit

Move the board to a new position / angle for each capture.
Aim for 20 captures covering corners, edges, tilts, and rotations.
"""

import cv2
import numpy as np
import sys
import os
import time
import threading

# ── Camera ────────────────────────────────────────────────────────────────────
RTSP_URL = "rtsp://admin:Admin123@192.168.2.12:554/live/0/SUB"

# ── Checkerboard ──────────────────────────────────────────────────────────────
BOARD_COLS     = 13   # 7 black squares on long side  → 14 squares → 13 inner corners
BOARD_ROWS     = 9    # 5 black squares on short side → 10 squares → 9 inner corners
SQUARE_SIZE_MM = 30.0

# ── Tuning ────────────────────────────────────────────────────────────────────
MIN_FRAMES       = 15
CAPTURE_COOLDOWN = 1.0    # seconds between captures
DETECT_SCALE     = 0.5    # run findChessboardCorners at this fraction of full res
DETECT_EVERY     = 2      # only run detection every N frames (1 = every frame)

SUBPIX_CRITERIA  = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

POSE_TIPS = [
    "Center of frame",      "Top-left corner",       "Top-right corner",
    "Bottom-left corner",   "Bottom-right corner",   "Tilt left",
    "Tilt right",           "Tilt up",               "Tilt down",
    "Rotate 45 CW",         "Rotate 45 CCW",         "Close-up",
    "Far away",             "Left edge",             "Right edge",
    "Top edge",             "Bottom edge",           "Diagonal tilt",
    "Slight rotation",      "Free pose",
]


# ─── Background frame grabber ─────────────────────────────────────────────────
# Reads frames in a daemon thread so the main loop always gets the latest frame
# and the RTSP buffer never backs up.
class FrameGrabber(threading.Thread):
    def __init__(self, url: str):
        super().__init__(daemon=True)
        self.url    = url
        self._frame = None
        self._lock  = threading.Lock()
        self._stop  = threading.Event()
        self.cap    = None

    def run(self):
        self.cap = _open_cap(self.url)
        while not self._stop.is_set():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self._lock:
                    self._frame = frame
            else:
                # brief pause then reconnect
                time.sleep(0.5)
                self.cap.release()
                self.cap = _open_cap(self.url)

    def get_frame(self):
        with self._lock:
            return self._frame

    def stop(self):
        self._stop.set()
        if self.cap:
            self.cap.release()


def _open_cap(url: str) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)          # keep buffer tiny → fresh frames
    cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 8000)
    cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000)
    return cap


def wait_for_first_frame(grabber: FrameGrabber, timeout: int = 12) -> np.ndarray:
    print(f"Connecting to camera...\n  {grabber.url}")
    deadline = time.time() + timeout
    while time.time() < deadline:
        f = grabber.get_frame()
        if f is not None:
            print(f"  Connected — {f.shape[1]}x{f.shape[0]}")
            return f
        time.sleep(0.2)
    print("ERROR: could not connect within timeout.")
    sys.exit(1)


# ─── Calibration helpers ──────────────────────────────────────────────────────
def make_objp(cols, rows, sq_mm):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= sq_mm
    return objp


def detect_board(gray_full: np.ndarray, board: tuple):
    """Detect at DETECT_SCALE resolution, refine corners at full resolution."""
    small = cv2.resize(gray_full, None, fx=DETECT_SCALE, fy=DETECT_SCALE)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
             cv2.CALIB_CB_NORMALIZE_IMAGE  |
             cv2.CALIB_CB_FAST_CHECK)
    found, corners_small = cv2.findChessboardCorners(small, board, flags)
    if not found:
        return False, None
    # Scale corners back to full resolution then refine
    corners_full = corners_small / DETECT_SCALE
    corners_full = cv2.cornerSubPix(
        gray_full, corners_full, (11, 11), (-1, -1), SUBPIX_CRITERIA
    )
    return True, corners_full


def calibrate_fisheye(obj_pts, img_pts, img_size):
    obj_f = [o.reshape(-1, 1, 3) for o in obj_pts]
    img_f = [p.reshape(-1, 1, 2) for p in img_pts]
    K = np.zeros((3, 3), dtype=np.float64)
    D = np.zeros((4, 1), dtype=np.float64)
    n = len(obj_f)
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(n)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(n)]
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    cv2.fisheye.calibrate(obj_f, img_f, img_size, K, D, rvecs, tvecs, flags)
    total = 0.0
    for i in range(n):
        proj, _ = cv2.fisheye.projectPoints(obj_f[i], rvecs[i], tvecs[i], K, D)
        total += cv2.norm(img_f[i], proj, cv2.NORM_L2) / len(proj)
    return K, D, total / n


def build_maps(K, D, img_size):
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, img_size, np.eye(3), balance=1.0
    )
    mx, my = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), new_K, img_size, cv2.CV_32FC1
    )
    return mx, my


def save_calibration(K, D, img_size, err, out_dir):
    path = os.path.join(out_dir, "calibration.npz")
    np.savez(path, camera_matrix=K, dist_coeffs=D,
             image_size=np.array(img_size), fisheye=True,
             mean_reprojection_error=err)
    sep = "=" * 56
    print(f"\n{sep}")
    print(f"  Saved : {path}")
    print(f"  RMS   : {err:.4f} px  ({'excellent' if err < 0.5 else 'good' if err < 1.0 else 'poor — recapture'})")
    print(f"{sep}")
    print("\nPaste into your undistortion script:\n")
    print(f"FISHEYE       = True")
    print(f"CAMERA_MATRIX = np.array({np.array2string(K, separator=', ')}, dtype=np.float64)")
    print(f"DIST_COEFFS   = np.array([{', '.join(f'{v:.8e}' for v in D.ravel())}], dtype=np.float64)")
    print(f"{sep}\n")


# ─── HUD ─────────────────────────────────────────────────────────────────────
def draw_hud(frame, n_cap, tip_idx, calibrated, show_undist, err):
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w, 50), (20, 20, 20), -1)
    progress = min(n_cap / MIN_FRAMES, 1.0)
    bar_w = int((w - 20) * progress)
    cv2.rectangle(frame, (10, 36), (w - 10, 48), (55, 55, 55), -1)
    cv2.rectangle(frame, (10, 36), (10 + bar_w, 48),
                  (0, 210, 70) if n_cap >= MIN_FRAMES else (0, 150, 255), -1)
    status = f"Frames: {n_cap}/{MIN_FRAMES}"
    if calibrated:
        status += f"   RMS:{err:.3f}px   Undist:{'ON' if show_undist else 'OFF(U)'}"
    cv2.putText(frame, status, (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.56, (230, 230, 230), 1, cv2.LINE_AA)
    if n_cap < len(POSE_TIPS):
        cv2.putText(frame, f"Next: {POSE_TIPS[tip_idx % len(POSE_TIPS)]}",
                    (10, h - 34), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (200, 200, 50), 1, cv2.LINE_AA)
    cv2.putText(frame, "SPACE=capture  C=calibrate  U=undistort  S=screenshot  Q=quit",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 150), 1, cv2.LINE_AA)


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    board     = (BOARD_COLS, BOARD_ROWS)
    objp_tmpl = make_objp(BOARD_COLS, BOARD_ROWS, SQUARE_SIZE_MM)
    out_dir   = os.path.dirname(os.path.abspath(__file__))

    grabber = FrameGrabber(RTSP_URL)
    grabber.start()
    wait_for_first_frame(grabber)

    obj_points, img_points = [], []
    K = D = map_x = map_y = None
    err         = 0.0
    calibrated  = False
    show_undist = False
    last_cap    = 0.0
    tip_idx     = 0
    frame_count = 0
    found       = False
    corners     = None

    cv2.namedWindow("ROV Camera Calibration", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ROV Camera Calibration", 900, 520)

    print("\nSPACE=capture  C=calibrate  U=undistort  S=screenshot  Q=quit")
    print("Move the board to a new position / angle for every capture.\n")

    while True:
        frame = grabber.get_frame()
        if frame is None:
            time.sleep(0.01)
            continue

        h, w = frame.shape[:2]
        frame_count += 1

        # ── Detect only every DETECT_EVERY frames ─────────────────────────────
        if frame_count % DETECT_EVERY == 0:
            gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = detect_board(gray, board)

        # ── Build display ─────────────────────────────────────────────────────
        display = frame.copy()

        if found and corners is not None:
            cv2.drawChessboardCorners(display, board, corners, True)
            cv2.putText(display, "Board detected — press SPACE",
                        (10, h - 56), cv2.FONT_HERSHEY_SIMPLEX,
                        0.62, (0, 255, 80), 2, cv2.LINE_AA)
        else:
            cv2.putText(display, "Looking for checkerboard...",
                        (10, h - 56), cv2.FONT_HERSHEY_SIMPLEX,
                        0.62, (0, 80, 255), 2, cv2.LINE_AA)

        if calibrated and show_undist and map_x is not None:
            undist  = cv2.remap(display, map_x, map_y, cv2.INTER_LINEAR)
            half_w, half_h = w // 2, h // 2
            display = np.hstack([
                cv2.resize(display, (half_w, half_h)),
                cv2.resize(undist,  (half_w, half_h)),
            ])
            cv2.putText(display, "ORIGINAL",    (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 80, 255), 2)
            cv2.putText(display, "UNDISTORTED", (half_w + 8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 80), 2)
        else:
            draw_hud(display, len(obj_points), tip_idx, calibrated, show_undist, err)

        cv2.imshow("ROV Camera Calibration", display)

        # ── Keys ──────────────────────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            now = time.time()
            if not found or corners is None:
                print("No board detected.")
            elif now - last_cap < CAPTURE_COOLDOWN:
                pass
            else:
                gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners_refined = cv2.cornerSubPix(
                    gray_full, corners.copy(), (11, 11), (-1, -1), SUBPIX_CRITERIA
                )
                obj_points.append(objp_tmpl)
                img_points.append(corners_refined)
                last_cap = now
                tip_idx += 1
                n = len(obj_points)
                left = max(0, MIN_FRAMES - n)
                print(f"  #{n:02d} captured"
                      + (f"  — {left} more needed" if left else "  — ready! press C"))
                flash = frame.copy()
                cv2.rectangle(flash, (0, 0), (w - 1, h - 1), (0, 255, 0), 14)
                cv2.imshow("ROV Camera Calibration", flash)
                cv2.waitKey(120)

        elif key in (ord('c'), ord('C')):
            n = len(obj_points)
            if n < MIN_FRAMES:
                print(f"Need {MIN_FRAMES} frames (have {n}).")
            else:
                print(f"\nCalibrating on {n} frames...")
                try:
                    K, D, err = calibrate_fisheye(obj_points, img_points, (w, h))
                    calibrated = True
                    map_x, map_y = build_maps(K, D, (w, h))
                    save_calibration(K, D, (w, h), err, out_dir)
                    print("Press U to preview undistortion.\n")
                except cv2.error as e:
                    print(f"Calibration failed: {e}")
                    print("Capture more frames from varied angles and try again.")

        elif key in (ord('u'), ord('U')):
            if not calibrated:
                print("Calibrate first (press C).")
            else:
                show_undist = not show_undist

        elif key in (ord('s'), ord('S')):
            fname = os.path.join(out_dir, f"screenshot_{int(time.time())}.jpg")
            cv2.imwrite(fname, display)
            print(f"Saved: {fname}")

        elif key in (ord('q'), ord('Q'), 27):
            break

    grabber.stop()
    cv2.destroyAllWindows()
    print("Done." if calibrated else "Quit without calibrating.")


if __name__ == "__main__":
    main()
