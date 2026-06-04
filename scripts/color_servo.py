"""
Color Visual Servo — Autonomous Red-Box Centering
==================================================
Uses the downward-facing camera to detect a red target box via HSV masking,
computes X/Y PID corrections, and drives the ROV thrusters to center the
red box in the frame.

Prerequisites:
  1. Arm the ROV via the web UI (localhost:3000) BEFORE running this script.
  2. Enable depth hold on the Pixhawk so Z is held automatically.

Usage:
    python3 color_servo.py [options]
    python3 color_servo.py --dry-run          # desk test — no UDP sent
    python3 color_servo.py --flip-x --gain 0.3  # after pool tuning

Controls (live window):
    p      pause / resume servo (sends neutral while paused)
    q/ESC  quit (always sends neutral packet before closing)
"""

import argparse
import json
import os
import socket
import sys
import time

import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
RTSP_URLS = [
    "rtsp://192.168.2.14:554/stream1",
    "rtsp://192.168.2.14:554/live",
    "rtsp://192.168.2.14:554/cam/realmonitor?channel=1&subtype=0",
    "rtsp://192.168.2.14:554/Streaming/Channels/101",
    "rtsp://192.168.2.14:554/h264Preview_01_main",
    "rtsp://admin:admin@192.168.2.14:554/stream1",
]

CALIBRATION_JSON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "backend", "calibration.json"
)

TEENSY_IP   = "192.168.2.177"
TEENSY_PORT = 5000

# Contours smaller than this (pixels²) are ignored as noise
MIN_CONTOUR_AREA = 500

# Error below this (fraction of half-frame) = "LOCKED" — PID still runs
LOCK_THRESHOLD = 0.05


# ── PID ───────────────────────────────────────────────────────────────────────

class PID:
    def __init__(self, kp, ki, kd, integral_limit=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._ilimit = integral_limit
        self._integral   = 0.0
        self._prev_error = 0.0
        self._last_t     = None

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._last_t     = None

    def update(self, error):
        now = time.time()
        dt  = (now - self._last_t) if self._last_t else 0.05
        dt  = max(dt, 1e-4)
        self._last_t = now

        self._integral = float(np.clip(
            self._integral + error * dt, -self._ilimit, self._ilimit
        ))
        d = (error - self._prev_error) / dt
        self._prev_error = error
        return self.kp * error + self.ki * self._integral + self.kd * d


# ── Camera ────────────────────────────────────────────────────────────────────

def _gst_rtsp_pipeline(url):
    """
    Low-latency GStreamer pipeline for Jetson.
    latency=0 removes the jitter buffer; appsink drop=true discards stale frames.
    Falls back to software decode (avdec_h264) if nvv4l2decoder is unavailable.
    """
    return (
        f"rtspsrc location={url} latency=0 protocols=tcp "
        "! rtph264depay ! h264parse "
        "! nvv4l2decoder enable-max-performance=1 "
        "! nvvidconv "
        "! video/x-raw,format=BGRx "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink max-buffers=1 drop=true sync=false"
    )


def _gst_rtsp_pipeline_sw(url):
    """Software-decode fallback (no Jetson HW required)."""
    return (
        f"rtspsrc location={url} latency=0 protocols=tcp "
        "! rtph264depay ! h264parse "
        "! avdec_h264 "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink max-buffers=1 drop=true sync=false"
    )


def try_connect(url, timeout=5):
    # Try Jetson HW-decode GStreamer pipeline first (lowest latency)
    for pipeline_fn in (_gst_rtsp_pipeline, _gst_rtsp_pipeline_sw):
        try:
            cap = cv2.VideoCapture(pipeline_fn(url), cv2.CAP_GSTREAMER)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    print(f"  Connected (GStreamer): {frame.shape[1]}x{frame.shape[0]}  {url}")
                    return cap
            cap.release()
        except Exception:
            pass

    # FFMPEG fallback
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, timeout * 1000)
    cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, timeout * 1000)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"  Connected (FFMPEG): {frame.shape[1]}x{frame.shape[0]}  {url}")
            return cap
    cap.release()
    return None


def open_local_camera(index):
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"  Local camera {index}: {frame.shape[1]}x{frame.shape[0]}")
            return cap
    cap.release()
    return None


# ── Color detection ───────────────────────────────────────────────────────────

def make_red_mask(frame, hue_low, hue_high):
    """Red wraps around 0° in HSV — merge two ranges: [0, hue_low] and [hue_high, 180]."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, np.array([0,        80,  80]),
                          np.array([hue_low,  255, 255]))
    m2 = cv2.inRange(hsv, np.array([hue_high, 80,  80]),
                          np.array([180,      255, 255]))
    mask = cv2.bitwise_or(m1, m2)
    k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def make_blue_mask(frame, hue_low, hue_high):
    """Detect the blue border box. Default HSV: H ∈ [hue_low, hue_high]."""
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([hue_low,  80,  50]),
                            np.array([hue_high, 255, 255]))
    k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def find_target(mask):
    """
    Return (cx, cy, bw, bh) — centre and size of largest red bounding box.
    Returns None if no valid contour found.
    """
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_CONTOUR_AREA:
        return None
    x, y, w, h = cv2.boundingRect(c)
    return (x + w // 2, y + h // 2, w, h)


# ── Thruster control ──────────────────────────────────────────────────────────

def load_dirs(path):
    try:
        with open(path) as f:
            d = json.load(f)["thrusterDirs"]
        dirs = [int(d[str(i)]) for i in range(1, 7)]
        print(f"  Thruster dirs loaded: {dirs}")
        return dirs
    except Exception as e:
        print(f"  WARNING: {e} — using defaults [-1, 1, -1, 1, 1, -1]")
        return [-1, 1, -1, 1, 1, -1]


def mix_and_send(sway, surge, dirs, gain, sock, ip, port, dry_run):
    """
    Mirror of server.js mixThrusters (yaw=roll=vertical=0).
    sway / surge are normalized [-1, 1] PID outputs.
    """
    x = float(np.clip(sway,  -1.0, 1.0))
    y = float(np.clip(surge, -1.0, 1.0))

    # X-frame horizontal mixing
    fl = -y + x
    fr = -y - x
    rl = -y - x
    rr = -y + x

    max_h = max(1.0, abs(fl), abs(fr), abs(rl), abs(rr))
    fl /= max_h; fr /= max_h; rl /= max_h; rr /= max_h

    normals = [fl, fr, rl, rr, 0.0, 0.0]   # verticals stay neutral

    pwms = [
        int(round(np.clip(1500 + v * dirs[i] * gain * 300, 1200, 1800)))
        for i, v in enumerate(normals)
    ]

    _send(pwms, sock, ip, port, dry_run)
    return pwms


def neutral(sock, ip, port, dry_run):
    _send([1500] * 6, sock, ip, port, dry_run)


def _send(pwms, sock, ip, port, dry_run):
    pkt = json.dumps({
        "type": "all",
        "pwms": pwms + [1500] * 10,   # 6 thrusters + 10 arm channels at neutral
        "ts":   int(time.time() * 1000)
    }).encode()
    if dry_run:
        print(f"  [DRY] pwms={pwms}")
    else:
        sock.sendto(pkt, (ip, port))


# ── PWM panel ────────────────────────────────────────────────────────────────

THRUSTER_LABELS = ("FL", "FR", "RL", "RR", "VL", "VR")

def draw_pwm_panel(canvas, pwms, panel_y):
    """Draw a dark strip below the camera feed showing each thruster's PWM bar."""
    h, w = canvas.shape[:2]
    panel_h = h - panel_y

    # Background
    cv2.rectangle(canvas, (0, panel_y), (w, h), (25, 25, 25), -1)
    cv2.line(canvas, (0, panel_y), (w, panel_y), (70, 70, 70), 1)

    n         = len(pwms)
    slot_w    = w // n
    center_y  = panel_y + panel_h // 2
    bar_max_h = max(4, (panel_h - 28) // 2)

    for i, (pwm, label) in enumerate(zip(pwms, THRUSTER_LABELS)):
        sx   = i * slot_w
        cx   = sx + slot_w // 2
        dev  = pwm - 1500          # -300 … +300

        # Bar
        bar_h = int(abs(dev) / 300 * bar_max_h)
        if dev > 5:
            color = (0, 210, 60)   # green  = forward thrust
            y1, y2 = center_y - bar_h, center_y
        elif dev < -5:
            color = (60, 60, 220)  # blue   = reverse thrust
            y1, y2 = center_y, center_y + bar_h
        else:
            color = (80, 80, 80)   # grey   = neutral
            y1, y2 = center_y - 2, center_y + 2

        cv2.rectangle(canvas, (cx - 10, min(y1, y2)), (cx + 10, max(y1, y2)), color, -1)

        # Zero line across slot
        cv2.line(canvas, (sx + 4, center_y), (sx + slot_w - 4, center_y), (60, 60, 60), 1)

        # Label (top of panel)
        lx = cx - (len(label) * 5)
        cv2.putText(canvas, label, (lx, panel_y + 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (190, 190, 190), 1)

        # PWM value (bottom of panel)
        vx = cx - 18
        cv2.putText(canvas, str(pwm), (vx, h - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Color visual servo — red box centering")
    parser.add_argument("--rtsp-url",   default=None)
    parser.add_argument("--teensy-ip",  default=TEENSY_IP)
    parser.add_argument("--kp",   type=float, default=0.4)
    parser.add_argument("--ki",   type=float, default=0.05)
    parser.add_argument("--kd",   type=float, default=0.1)
    parser.add_argument("--gain", type=float, default=0.35,
                        help="Overall thrust scale 0–1 (start conservative at 0.35)")
    parser.add_argument("--hue-low",  type=int, default=10,
                        help="Upper edge of first red hue range [0–hue-low] (default 10)")
    parser.add_argument("--hue-high", type=int, default=170,
                        help="Lower edge of second red hue range [hue-high–180] (default 170)")
    parser.add_argument("--blue-hue-low",  type=int, default=100,
                        help="Lower blue hue (default 100)")
    parser.add_argument("--blue-hue-high", type=int, default=130,
                        help="Upper blue hue (default 130)")
    parser.add_argument("--min-blue-area", type=int, default=1000,
                        help="Minimum blue pixel area to trigger correction (default 1000)")
    parser.add_argument("--flip-x",  action="store_true",
                        help="Invert sway correction (if ROV drifts wrong way left/right)")
    parser.add_argument("--flip-y",  action="store_true",
                        help="Invert surge correction (if ROV drifts wrong way forward/back)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print UDP packets without sending — safe for desk testing")
    parser.add_argument("--camera", type=int, default=None,
                        help="Local webcam index (0, 1, ...) — use instead of RTSP for land testing")
    args = parser.parse_args()

    print("=" * 60)
    print("  COLOR VISUAL SERVO — Red Box Centering")
    print("=" * 60)
    print()
    print("  IMPORTANT: Arm the ROV via web UI (localhost:3000)")
    print("  and enable depth hold BEFORE running this script.")
    print()

    dirs = load_dirs(CALIBRATION_JSON)

    # Connect camera
    cap = None
    if args.camera is not None:
        print(f"Opening local camera {args.camera} ...")
        cap = open_local_camera(args.camera)
    elif args.rtsp_url:
        print(f"Connecting to {args.rtsp_url} ...")
        cap = try_connect(args.rtsp_url)
    else:
        print("Auto-detecting camera on 192.168.2.14 ...")
        for url in RTSP_URLS:
            cap = try_connect(url)
            if cap:
                break
    if cap is None:
        print("ERROR: Could not connect to camera.")
        print("  Land test:  --camera 0  (built-in webcam)")
        print("  ROV:        --rtsp-url rtsp://192.168.2.14:554/stream1")
        sys.exit(1)

    ret, frame = cap.read()
    if not ret:
        sys.exit("ERROR: Could not read first frame.")
    fh, fw = frame.shape[:2]
    fcx, fcy = fw // 2, fh // 2

    pid_x = PID(args.kp, args.ki, args.kd)
    pid_y = PID(args.kp, args.ki, args.kd)

    sock = None if args.dry_run else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sx = -1 if args.flip_x else 1
    sy = -1 if args.flip_y else 1
    paused = False
    last_pwms = [1500] * 6   # persists between frames for the PWM panel

    print(f"\n  PID Kp={args.kp}  Ki={args.ki}  Kd={args.kd}  Gain={args.gain}")
    print(f"  Hue ranges: [0–{args.hue_low}] and [{args.hue_high}–180]")
    print(f"  {'DRY RUN — no UDP sent' if args.dry_run else f'Sending to {args.teensy_ip}:{TEENSY_PORT}'}")
    print("  Controls: p=pause  q=quit\n")

    cv2.namedWindow("Color Servo", cv2.WINDOW_NORMAL)

    try:
        while True:
            # Drain any buffered frames so we always get the latest one
            cap.grab()
            cap.grab()
            ret, frame = cap.retrieve()
            if not ret or frame is None:
                print("Stream lost — retrying ...")
                time.sleep(1)
                continue

            red_mask  = make_red_mask(frame, args.hue_low, args.hue_high)
            blue_mask = make_blue_mask(frame, args.blue_hue_low, args.blue_hue_high)
            target    = find_target(red_mask)

            blue_area    = cv2.countNonZero(blue_mask)
            blue_visible = blue_area > args.min_blue_area

            display = frame.copy()
            cv2.drawMarker(display, (fcx, fcy), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)

            # Highlight detected blue region on display
            blue_overlay = np.zeros_like(display)
            blue_overlay[blue_mask > 0] = (200, 100, 0)   # blue tint
            cv2.addWeighted(blue_overlay, 0.35, display, 1.0, 0, display)

            sway = surge = 0.0
            err_x = err_y = 0.0

            if target is not None:
                cx, cy, bw, bh = target
                err_x = (cx - fcx) / (fw / 2.0)
                err_y = (cy - fcy) / (fh / 2.0)

                # Draw red bounding box and centroid always
                cv2.rectangle(display,
                              (cx - bw // 2, cy - bh // 2),
                              (cx + bw // 2, cy + bh // 2),
                              (0, 255, 0), 2)
                cv2.drawMarker(display, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)

                if blue_visible:
                    # Blue in frame → ROV off-centre → show error arrow and correct
                    cv2.arrowedLine(display, (fcx, fcy), (cx, cy), (255, 140, 0), 2, tipLength=0.2)

            cv2.putText(display,
                        f"blue px={blue_area}  err x={err_x:+.3f}  y={err_y:+.3f}"
                        f"  sway={sway:+.2f}  surge={surge:+.2f}",
                        (10, fh - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            # ── Decision: correct only when blue is visible ───────────────────
            if paused:
                pid_x.reset(); pid_y.reset()
                last_pwms = [1500] * 6
                neutral(sock, args.teensy_ip, TEENSY_PORT, args.dry_run)

            elif blue_visible and target is not None:
                # Blue detected + red found → PID correction
                sway  = float(np.clip(pid_x.update(err_x), -1, 1)) * sx
                surge = float(np.clip(pid_y.update(err_y), -1, 1)) * sy
                last_pwms = mix_and_send(sway, surge, dirs, args.gain,
                                         sock, args.teensy_ip, TEENSY_PORT, args.dry_run)

            else:
                # Only red visible (no blue) OR no target at all → hold neutral
                pid_x.reset(); pid_y.reset()
                last_pwms = [1500] * 6
                neutral(sock, args.teensy_ip, TEENSY_PORT, args.dry_run)

            # HUD status
            if paused:
                label, color = "PAUSED",          (100, 100, 255)
            elif not blue_visible and target is not None:
                label, color = "CENTERED — HOLD", (0, 220, 0)
            elif blue_visible and target is not None:
                label, color = "CORRECTING",      (0, 200, 255)
            elif blue_visible and target is None:
                label, color = "LOST — NO RED",   (0, 80, 255)
            else:
                label, color = "NO TARGET",       (0, 80, 255)

            cv2.putText(display, label, (10, 38),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 2)

            # Extend canvas downward and draw PWM bar panel
            panel_h = 75
            canvas = np.zeros((fh + panel_h, fw, 3), dtype=np.uint8)
            canvas[:fh] = display
            draw_pwm_panel(canvas, last_pwms, fh)

            cv2.imshow("Color Servo", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('p'), ord('P')):
                paused = not paused
                if paused:
                    pid_x.reset(); pid_y.reset()
                print(f"  {'Paused' if paused else 'Resumed'}")

    finally:
        print("  Sending neutral thrust and closing ...")
        neutral(sock, args.teensy_ip, TEENSY_PORT, args.dry_run)
        if sock:
            sock.close()
        cap.release()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
