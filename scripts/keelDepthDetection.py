import argparse
import cv2
import numpy as np

# ---------- CONFIG ----------
REF_LENGTH_CM = 15.0   # known reference length in cm (corner marker)
NUM_REF_LINES = 1      # number of reference lines to average (more = more accurate)
PIPE_OD_MM    = 21.3   # 1/2" sch40 PVC OD, used by the diameter cross-check
# ----------------------------
# Scoring reminder:  error <= 5 cm -> 10 pts ;  5.01-10 cm -> 5 pts


class KeelMeasurer:
    """
    Phase 1 – click both ends of the known reference line NUM_REF_LINES times.
               The px/cm scale is the average across all reference lines.
    Phase 2 – click a polyline down the keel (top to bottom).
    """

    def __init__(self, img):
        self.base      = img.copy()
        self.img       = img.copy()
        self.ref_lines = []    # list of completed (pt1, pt2) reference pairs
        self.cur_ref   = []    # 0 or 1 point for the in-progress reference line
        self.keel_pts  = []
        self.px_per_cm = None
        self.mode      = "ref"

    # ------------------------------------------------------------------
    def _recompute_scale(self):
        if not self.ref_lines:
            self.px_per_cm = None
            return
        scales = [
            np.hypot(p2[0] - p1[0], p2[1] - p1[1]) / REF_LENGTH_CM
            for p1, p2 in self.ref_lines
        ]
        self.px_per_cm = float(np.mean(scales))

    # ------------------------------------------------------------------
    def click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.mode == "ref":
            self.cur_ref.append((x, y))
            if len(self.cur_ref) == 2:
                self.ref_lines.append((self.cur_ref[0], self.cur_ref[1]))
                self.cur_ref = []
                self._recompute_scale()
                if len(self.ref_lines) >= NUM_REF_LINES:
                    self.mode = "keel"
        else:
            self.keel_pts.append((x, y))

        self.redraw()

    # ------------------------------------------------------------------
    def undo(self):
        """Undo last action in whichever phase is active."""
        if self.mode == "keel" and self.keel_pts:
            self.keel_pts.pop()
        elif self.cur_ref:
            self.cur_ref.pop()
        elif self.ref_lines:
            self.ref_lines.pop()
            self.mode = "ref"
            self._recompute_scale()
        self.redraw()

    # ------------------------------------------------------------------
    def keel_length_cm(self):
        if self.px_per_cm is None or len(self.keel_pts) < 2:
            return None
        total_px = sum(
            np.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(self.keel_pts, self.keel_pts[1:])
        )
        return total_px / self.px_per_cm

    # ------------------------------------------------------------------
    def redraw(self):
        self.img = self.base.copy()
        h, w = self.img.shape[:2]

        # --- draw completed reference lines ---
        ref_colors = [(0, 80, 255), (0, 180, 255), (0, 255, 220)]
        for i, (p1, p2) in enumerate(self.ref_lines):
            c   = ref_colors[i % len(ref_colors)]
            px  = np.hypot(p2[0] - p1[0], p2[1] - p1[1])
            s   = px / REF_LENGTH_CM
            mid = tuple(np.mean([p1, p2], axis=0).astype(int))
            cv2.line(self.img, p1, p2, c, 2)
            cv2.circle(self.img, p1, 5, c, -1)
            cv2.circle(self.img, p2, 5, c, -1)
            cv2.putText(self.img,
                        f"ref{i+1}: {REF_LENGTH_CM:.0f}cm  {s:.2f}px/cm",
                        (mid[0] + 6, mid[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, c, 1)

        # --- draw in-progress reference point ---
        for p in self.cur_ref:
            cv2.circle(self.img, p, 5, (0, 60, 200), -1)

        # --- draw keel polyline ---
        for p in self.keel_pts:
            cv2.circle(self.img, p, 4, (0, 255, 60), -1)
        for a, b in zip(self.keel_pts, self.keel_pts[1:]):
            cv2.line(self.img, a, b, (0, 255, 60), 2)
        if len(self.keel_pts) >= 2:
            cv2.circle(self.img, self.keel_pts[0],  6, (255, 255,  0), 2)  # top marker
            cv2.circle(self.img, self.keel_pts[-1], 6, (255, 100,  0), 2)  # bottom marker

        # --- status bar ---
        L = self.keel_length_cm()
        done_refs = len(self.ref_lines)

        if L is not None:
            avg_scale = f"{self.px_per_cm:.2f}px/cm (avg {NUM_REF_LINES} refs)"
            msg = f"KEEL DEPTH: {L:.1f} cm  =  {L/100:.3f} m    [{avg_scale}]"
        elif self.mode == "ref":
            if self.cur_ref:
                msg = (f"click 2nd end of reference {done_refs+1}/{NUM_REF_LINES}"
                       f"  ({REF_LENGTH_CM:.0f} cm)")
            else:
                msg = (f"click 1st end of reference {done_refs+1}/{NUM_REF_LINES}"
                       f"  ({REF_LENGTH_CM:.0f} cm known)")
        else:
            avg_scale = f"{self.px_per_cm:.2f}px/cm"
            msg = f"click down the keel top→bottom    [scale: {avg_scale}]"

        cv2.rectangle(self.img, (0, 0), (w, 32), (0, 0, 0), -1)
        cv2.putText(self.img, msg, (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)


# ----------------------------------------------------------------------
def diameter_cross_check(img):
    """Sanity-check: click pipe width at top (L, R) then bottom (L, R).
    If the ratio top/bot differs from 1.0 there is perspective foreshortening."""
    pts = []

    def cb(e, x, y, flags, p):
        if e == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x, y))

    win = "diameter check – 4 clicks: top L,R then bottom L,R  (q=done)"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, cb)
    while True:
        show = img.copy()
        for p in pts:
            cv2.circle(show, p, 4, (255, 80, 0), -1)
        cv2.imshow(win, show)
        if cv2.waitKey(30) & 0xFF == ord('q') or len(pts) == 4:
            break
        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) == 0:
            break
    cv2.destroyWindow(win)
    if len(pts) == 4:
        top_px = np.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1])
        bot_px = np.hypot(pts[2][0] - pts[3][0], pts[2][1] - pts[3][1])
        ratio  = top_px / bot_px
        print(f"top width  {top_px:.1f}px  ->  {PIPE_OD_MM / top_px:.3f} mm/px")
        print(f"bot width  {bot_px:.1f}px  ->  {PIPE_OD_MM / bot_px:.3f} mm/px")
        print(f"top/bot ratio = {ratio:.3f}  (1.000 = no perspective error)")
        if abs(ratio - 1.0) > 0.05:
            print("WARNING: >5% perspective distortion – measurement will be off")


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
            k = cv2.waitKey(30) & 0xFF
            if k == ord(' '):
                img = f.copy()
                break
            if k == ord('q') or cv2.getWindowProperty("feed", cv2.WND_PROP_VISIBLE) < 1:
                cap.release()
                cv2.destroyAllWindows()
                return
        cap.release()
        cv2.destroyWindow("feed")

    if img is None:
        print("no frame captured")
        return

    print(f"Reference: {REF_LENGTH_CM:.0f} cm  |  averaging {NUM_REF_LINES} reference line(s)")
    print("Keys:  z=undo  r=reset  c=diameter cross-check  q=quit & print result")

    m = KeelMeasurer(img)
    cv2.namedWindow("measure", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("measure", m.click)
    m.redraw()

    while True:
        cv2.imshow("measure", m.img)
        if cv2.getWindowProperty("measure", cv2.WND_PROP_VISIBLE) == 0:
            break
        k = cv2.waitKey(30) & 0xFF

        if k == ord('r'):
            m = KeelMeasurer(img)
            cv2.setMouseCallback("measure", m.click)
            m.redraw()

        elif k == ord('z'):
            m.undo()

        elif k == ord('c'):
            diameter_cross_check(img)

        elif k == ord('q'):
            L = m.keel_length_cm()
            if L is not None:
                print(f"\nfinal keel depth = {L:.1f} cm  ({L/100:.3f} m)")
                if m.px_per_cm:
                    print(f"scale used       = {m.px_per_cm:.3f} px/cm  "
                          f"(avg of {len(m.ref_lines)} reference line(s))")
            else:
                print("measurement incomplete")
            break

    cv2.destroyAllWindows()


# ----------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Keel depth measurement tool")
    parser.add_argument("--image",    default=None,
                        help="path to an image file")
    parser.add_argument("--rtsp-url",
                        default="rtsp://admin:Admin123@192.168.2.16:554/live/0/SUB",
                        help="RTSP stream URL (default: cam2)")
    args = parser.parse_args()

    if args.image:
        run(image_path=args.image)
    else:
        run(source=args.rtsp_url)
