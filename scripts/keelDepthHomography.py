import argparse
import cv2
import numpy as np

# ---------- CONFIG ----------
REF_LENGTH_CM = 15.0    # known peg length for corners TL, TR, BR
EPSILON       = 1.0     # pixel shift used for numerical Jacobian
DEFAULT_RTSP  = "rtsp://admin:Admin123@192.168.2.13:554/live/0/SUB"
CORNER_LABELS = ["TL", "TR", "BR", "BL"]
KNOWN_INDICES = [0, 1, 2]   # indices with known peg length
UNKNOWN_INDEX = 3            # BL corner — keel depth to find
# ----------------------------
# Click order: TL → TR → BR → BL
# Phase 0 (SQUARE): click the 4 corners of the PVC square frame
# Phase 1 (TIPS)  : click the tip of each peg in the same TL→TR→BR→BL order
# Result is computed automatically after the 4th tip click.

CORNER_COLORS = [
    (0,  80, 255),    # TL — blue
    (0, 180, 255),    # TR — light blue
    (0, 220, 180),    # BR — teal
    (255, 80,   0),   # BL — orange (unknown)
]


class HomographyKeelMeasurer:
    def __init__(self, img):
        self.base        = img.copy()
        self.img         = img.copy()
        self.corners     = []     # up to 4 (x,y) corner points
        self.tips        = []     # up to 4 (x,y) tip points
        self.H           = None   # 3x3 homography, image → unit square
        self.result_cm   = None   # final keel depth
        self.per_peg_cm  = []     # measured cm for all 4 pegs (set in DONE)
        self.mode        = "SQUARE"   # "SQUARE" | "TIPS" | "DONE"

    # ------------------------------------------------------------------
    def click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.mode == "SQUARE" and len(self.corners) < 4:
            self.corners.append((x, y))
            if len(self.corners) == 4:
                self.H    = self._compute_homography()
                self.mode = "TIPS"
        elif self.mode == "TIPS" and len(self.tips) < 4:
            self.tips.append((x, y))
            if len(self.tips) == 4:
                self.result_cm = self._compute()
                self.mode      = "DONE"
        self.redraw()

    # ------------------------------------------------------------------
    def undo(self):
        if self.mode == "DONE":
            self.tips.pop()
            self.result_cm  = None
            self.per_peg_cm = []
            self.mode       = "TIPS"
        elif self.mode == "TIPS":
            if self.tips:
                self.tips.pop()
            else:
                self.corners.pop()
                self.H    = None
                self.mode = "SQUARE"
        elif self.mode == "SQUARE" and self.corners:
            self.corners.pop()
        self.redraw()

    # ------------------------------------------------------------------
    def _apply_H(self, H, pt):
        p = np.float64([pt[0], pt[1], 1.0])
        w = H @ p
        return np.array([w[0] / w[2], w[1] / w[2]])

    def _compute_homography(self):
        src = np.float32(self.corners)
        dst = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
        H, _ = cv2.findHomography(src, dst)
        return H

    def _local_scale(self, pt):
        """sqrt(|det(Jacobian of H at pt)|) — world-units-per-pixel at this image point."""
        H, eps = self.H, EPSILON
        x, y   = float(pt[0]), float(pt[1])
        dWdPx  = (self._apply_H(H, (x + eps, y)) - self._apply_H(H, (x - eps, y))) / (2 * eps)
        dWdPy  = (self._apply_H(H, (x, y + eps)) - self._apply_H(H, (x, y - eps))) / (2 * eps)
        J      = np.column_stack([dWdPx, dWdPy])   # 2×2
        return float(np.sqrt(abs(np.linalg.det(J))))

    def _compute(self):
        corrected = []
        for i in range(4):
            px_len     = np.hypot(self.tips[i][0] - self.corners[i][0],
                                  self.tips[i][1] - self.corners[i][1])
            scale      = self._local_scale(self.corners[i])
            corrected.append(px_len * scale)

        # calibrate from the 3 known pegs
        K = np.mean([corrected[i] for i in KNOWN_INDICES]) / REF_LENGTH_CM
        self.per_peg_cm = [corrected[i] / K for i in range(4)]
        keel_cm         = self.per_peg_cm[UNKNOWN_INDEX]

        print("\n--- Homography Keel Depth Analysis ---")
        for i in KNOWN_INDICES:
            err = self.per_peg_cm[i] - REF_LENGTH_CM
            print(f"  Peg {i} ({CORNER_LABELS[i]}):  {self.per_peg_cm[i]:.2f} cm "
                  f"(expected {REF_LENGTH_CM:.1f} cm,  err {err:+.2f} cm)")
        print(f"  Keel depth ({CORNER_LABELS[UNKNOWN_INDEX]}):  "
              f"{keel_cm:.2f} cm  =  {keel_cm / 100:.4f} m")
        print("--------------------------------------\n")
        return keel_cm

    # ------------------------------------------------------------------
    def redraw(self):
        self.img    = self.base.copy()
        h, w        = self.img.shape[:2]
        n_corners   = len(self.corners)
        n_tips      = len(self.tips)

        # --- square outline ---
        if n_corners == 4:
            pts = np.array(self.corners, dtype=np.int32)
            cv2.polylines(self.img, [pts], isClosed=True, color=(200, 200, 200), thickness=1)

        # --- corners ---
        for i, c in enumerate(self.corners):
            col = CORNER_COLORS[i]
            cv2.circle(self.img, c, 6, col, -1)
            cv2.putText(self.img, CORNER_LABELS[i],
                        (c[0] + 8, c[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, col, 2)

        # --- peg lines and tips ---
        for i, t in enumerate(self.tips):
            col = CORNER_COLORS[i]
            cv2.line(self.img, self.corners[i], t, col, 2)
            cv2.circle(self.img, t, 5, col, -1)

            # annotation label
            if self.per_peg_cm:
                mid = (
                    (self.corners[i][0] + t[0]) // 2 + 6,
                    (self.corners[i][1] + t[1]) // 2 - 6,
                )
                if i in KNOWN_INDICES:
                    err   = self.per_peg_cm[i] - REF_LENGTH_CM
                    label = f"{self.per_peg_cm[i]:.1f}cm ({err:+.1f})"
                    if   abs(err) <= 2.0: ann_col = (0, 220,  50)
                    elif abs(err) <= 5.0: ann_col = (0, 160, 255)
                    else:                  ann_col = (0,  40, 220)
                    cv2.putText(self.img, label, mid,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, ann_col, 1)
                else:
                    label = f"KEEL: {self.per_peg_cm[i]:.1f}cm"
                    cv2.putText(self.img, label, mid,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            else:
                if i < 3:
                    cv2.putText(self.img, f"{REF_LENGTH_CM:.0f}cm",
                                (t[0] + 5, t[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, CORNER_COLORS[i], 1)
                else:
                    cv2.putText(self.img, "?",
                                (t[0] + 5, t[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, CORNER_COLORS[i], 2)

        # --- status bar (top) ---
        if self.mode == "SQUARE":
            if n_corners < 4:
                suffix = "  (this is the UNKNOWN peg)" if n_corners == 3 else ""
                msg = (f"[SQUARE {n_corners + 1}/4]  Click {CORNER_LABELS[n_corners]} "
                       f"corner of PVC frame{suffix}")
            else:
                msg = "All 4 corners set — now click peg tips"
        elif self.mode == "TIPS":
            known_str = f"known = {REF_LENGTH_CM:.0f} cm" if n_tips in KNOWN_INDICES else "UNKNOWN — keel depth"
            msg = (f"[TIPS {n_tips + 1}/4]  Click TIP of peg at "
                   f"{CORNER_LABELS[n_tips]}  ({known_str})")
        else:
            errs    = [abs(self.per_peg_cm[i] - REF_LENGTH_CM) for i in KNOWN_INDICES]
            max_err = max(errs)
            msg     = (f"KEEL DEPTH: {self.result_cm:.1f} cm  ({self.result_cm / 100:.3f} m)    "
                       f"ref max-err = {max_err:.1f} cm    [z=undo  r=reset  q=quit]")

        cv2.rectangle(self.img, (0, 0), (w, 32), (0, 0, 0), -1)
        cv2.putText(self.img, msg, (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)

        # --- result bar (bottom, DONE only) ---
        if self.mode == "DONE":
            cv2.rectangle(self.img, (0, h - 36), (w, h), (0, 0, 0), -1)
            cv2.putText(self.img,
                        f"KEEL: {self.result_cm:.2f} cm  =  {self.result_cm / 100:.4f} m",
                        (8, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)


# ----------------------------------------------------------------------
def run(image_path=None, source=None):
    if image_path:
        img = cv2.imread(image_path)
        if img is None:
            print(f"ERROR: could not read image: {image_path}")
            return
    else:
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            print(f"ERROR: could not open source: {source}")
            return
        print("SPACE = grab frame  |  q = quit feed")
        img = None
        while True:
            ok, f = cap.read()
            if not ok:
                break
            cv2.imshow("feed", f)
            k = cv2.waitKey(1) & 0xFF
            if k == ord(' '):
                img = f.copy()
                break
            if k == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                return
        cap.release()
        cv2.destroyWindow("feed")

    if img is None:
        print("no frame captured")
        return

    print(f"Reference peg length: {REF_LENGTH_CM:.0f} cm  (corners TL, TR, BR)")
    print("Click order:  TL → TR → BR → BL  (BL = unknown keel depth)")
    print("Keys:  z=undo  r=reset  q=quit & print result")

    m = HomographyKeelMeasurer(img)
    cv2.namedWindow("measure", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("measure", m.click)
    m.redraw()

    while True:
        cv2.imshow("measure", m.img)
        k = cv2.waitKey(1) & 0xFF

        if k == ord('r'):
            m = HomographyKeelMeasurer(img)
            cv2.setMouseCallback("measure", m.click)
            m.redraw()
        elif k == ord('z'):
            m.undo()
        elif k == ord('q'):
            if m.result_cm is not None:
                print(f"\nFINAL keel depth = {m.result_cm:.2f} cm  ({m.result_cm / 100:.4f} m)")
            else:
                print("measurement incomplete")
            break

    cv2.destroyAllWindows()


# ----------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Homography-corrected keel depth measurement")
    parser.add_argument("--image",    default=None,
                        help="path to an image file")
    parser.add_argument("--rtsp-url", default=DEFAULT_RTSP,
                        help="RTSP stream URL")
    args = parser.parse_args()

    if args.image:
        run(image_path=args.image)
    else:
        run(source=args.rtsp_url)
