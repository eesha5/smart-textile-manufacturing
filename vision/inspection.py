"""
inspection.py Smart Textile Manufacturing - FINAL
CAMERA -> OPENCV PRODUCT DETECTION -> GOOD/DEFECT CLASSIFICATION -> KPIs -> robot sorting
Orthographic or perspective vision sensor. Image: sim.getVisionSensorImg() only.
Defect rules (configurable): 1) defect AREA on the product surface (stains, holes, patches)
                              2) colour deviation from the golden reference learned from Product_A
Counters/KPIs start at 0 on every Play and are driven only by the vision verdict.
Keys: q quit | s snapshot | d debug mask | r reset counters
Outputs (inspection_log/): results.csv, kpi_summary.txt, final_dashboard.png, one PNG per product
"""
import csv
import math
import os
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ============================ CONFIG ============================
CAMERA_PATH = "/Inspection_Camera"
TRIGGER_PATH = "/Sensor_InspectionTrigger"
PRODUCT_PATHS = ("/Product_A", "/Product_B")
KNOWN_GOOD = "Product_A"
STATION_Z = 0.30

ACTIVE_SIGNAL = "InspectionActive"       # set by Robot_Controller during State 5
RESULT_SIGNAL = "InspectionResult"       # -1 waiting, 0 GOOD, 1 DEFECT
STATE_SIGNAL = "RobotState"              # for the flow strip
PACKAGED_SIGNAL = "PackagedCount"
PUBLISH_SIGNAL = True
PUBLISH_KPI_SIGNALS = True               # KPI_Good, KPI_Defect, KPI_Total, KPI_DefectPct, ...
GATE_XY_TOL = 0.06
GATE_Z_TOL = 0.08

IMG_X_SIGN = -1                          # image column = w/2 + IMG_X_SIGN * camera X (verified)
IMG_Y_SIGN = -1

IGNORE_HSV = [((3, 190, 100), (22, 255, 255))]   # ABB robot / orange plate
ROI_PAD_PX = 3
RING_PX = 14
MIN_VISIBLE_FRAC = 0.35
MIN_BG_SEP = 25.0
MIN_FACE_FRAC = 0.25

FACE_TOL = 14.0
INNER_MARGIN_FRAC = 0.12
SPOT_DIST = 28.0                         # colour distance that marks a pixel as anomalous
MIN_BLOB_PX = 10
DEFECT_AREA_PCT = 1.0                    # DEFECT if anomalous area >= this % of the product surface
MIN_DEFECT_PX = 20
TEACH_GOLDEN = True                      # learn the reference colour from the first Product_A
GOOD_REF_BGR = None                      # or fix it, e.g. (255, 208, 74)
COLOUR_TOL = 22.0                        # DEFECT if surface colour is this far from the reference

STABLE_FRAMES = 5
HOLD_SEC = 3.0
WINDOW = "Inspection Camera - OpenCV"
SAVE_DIR = "inspection_log"
# ================================================================

FONT = cv2.FONT_HERSHEY_SIMPLEX
C_BG, C_PANEL, C_LINE = (22, 22, 22), (40, 40, 40), (85, 85, 85)
C_TXT, C_DIM = (240, 240, 240), (165, 165, 165)
C_GOOD, C_BAD, C_WARN, C_HEAD = (60, 190, 60), (70, 70, 235), (0, 190, 255), (255, 220, 0)
CAM_PX, PANEL_W, HEADER_H, STRIP_H = 512, 560, 48, 44
CANVAS_W, CANVAS_H = CAM_PX + PANEL_W, HEADER_H + STRIP_H + CAM_PX
STATUS_FILL = {"DETECTED": (0, 140, 0), "WAITING": (0, 110, 150), "NOT VISIBLE": (0, 90, 200)}
RESULT_FILL = {"GOOD": (0, 150, 0), "DEFECT": (40, 40, 200), "ANALYSING": (0, 110, 150),
               "CALIBRATING": (150, 90, 0), "WAITING": (70, 70, 70)}
STAGES = ["RAW MATERIAL", "AUTO FEEDING", "PROCESS 1", "PROCESS 2",
          "INSPECTION", "DEFECT DETECTION", "SORTING", "PACKAGING"]
STATE_TO_STAGES = {0: [0, 1], 1: [1], 2: [1], 3: [2], 4: [3], 5: [4, 5],
                   6: [6], 7: [6], 8: [], 9: [7]}


# ----------------------------- helpers -----------------------------
def param(fn, h, pid):
    r = fn(h, pid)
    return r[-1] if isinstance(r, (list, tuple)) else r


def grab(sim, cam):
    buf, res = sim.getVisionSensorImg(cam)
    w, h = int(res[0]), int(res[1])
    img = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 3)
    return cv2.cvtColor(cv2.flip(img, 0), cv2.COLOR_RGB2BGR)


def dominant(pix):
    q = (pix // 12).astype(np.int32)
    key = q[:, 0] * 10000 + q[:, 1] * 100 + q[:, 2]
    vals, counts = np.unique(key, return_counts=True)
    return np.median(pix[key == vals[counts.argmax()]], axis=0)


def fmt_time(sec):
    sec = int(max(0, sec))
    return "%02d:%02d" % (sec // 60, sec % 60)


def fv(v, fmt, unit=""):
    return "--" if v is None else (fmt % v) + unit


# ----------------------------- scene geometry -----------------------------
class Scene:
    def __init__(self, sim):
        self.sim = sim
        self.cam = sim.getObject(CAMERA_PATH)
        self.trig = self._get(TRIGGER_PATH)
        self.prods = {}
        for pth in PRODUCT_PATHS:
            h = self._get(pth)
            if h is not None:
                self.prods[pth.strip("/")] = (h, self._size(h))
        self.ortho, self.ortho_size, self.fov = False, 1.0, math.radians(60.0)
        self.trig_pos = None
        self._R = self._t = None
        self._pose_t = 0.0

    def _get(self, path):
        try:
            return self.sim.getObject(path)
        except Exception:
            print("WARNING: object not found:", path)
            return None

    def _size(self, h):
        sim = self.sim
        try:
            return [float(v) for v in sim.getShapeBB(h)]
        except Exception:
            pass
        try:
            g = lambda pid: param(sim.getObjectFloatParam, h, pid)
            return [g(sim.objfloatparam_objbbox_max_x) - g(sim.objfloatparam_objbbox_min_x),
                    g(sim.objfloatparam_objbbox_max_y) - g(sim.objfloatparam_objbbox_min_y),
                    g(sim.objfloatparam_objbbox_max_z) - g(sim.objfloatparam_objbbox_min_z)]
        except Exception:
            return None

    def _refresh(self):
        sim = self.sim
        now = time.time()
        if self._R is not None and now - self._pose_t < 1.0:
            return
        M = sim.getObjectMatrix(self.cam, -1)
        self._R = np.array([[M[0], M[1], M[2]], [M[4], M[5], M[6]], [M[8], M[9], M[10]]])
        self._t = np.array([M[3], M[7], M[11]])
        try:
            self.ortho = int(param(sim.getObjectInt32Param, self.cam,
                                   sim.visionintparam_perspective_operation)) == 0
        except Exception:
            self.ortho = False
        if self.ortho:
            sid = getattr(sim, "visionfloatparam_ortho_size", None)
            if sid is not None:
                self.ortho_size = param(sim.getObjectFloatParam, self.cam, sid)
        else:
            fov = param(sim.getObjectFloatParam, self.cam, sim.visionfloatparam_perspective_angle)
            self.fov = fov if fov < 3.2 else math.radians(fov)
        if self.trig is not None:
            self.trig_pos = sim.getObjectPosition(self.trig, -1)
        self._pose_t = now

    def geometry(self, w, h):
        sim = self.sim
        self._refresh()
        g = dict(roi=None, poly=None, name=None, at_station=False, need=None, full_view=None,
                 ortho=self.ortho, cur=self.ortho_size,
                 view_txt=("ortho size %.2f m" % self.ortho_size) if self.ortho
                 else ("FOV %.0f deg" % math.degrees(self.fov)))
        if self.trig_pos is None or not self.prods:
            return g
        tp = np.array([self.trig_pos[0], self.trig_pos[1], STATION_Z])

        found = None
        for name, (ph, size) in self.prods.items():
            M = sim.getObjectMatrix(ph, -1)
            pos = np.array([M[3], M[7], M[11]])
            if (math.hypot(pos[0] - tp[0], pos[1] - tp[1]) <= GATE_XY_TOL
                    and abs(pos[2] - STATION_Z) <= GATE_Z_TOL):
                found = (name, M, size)
                break
        if found is not None and found[2] is not None:
            name, M, size = found
            Rm = np.array([[M[0], M[1], M[2]], [M[4], M[5], M[6]], [M[8], M[9], M[10]]])
            g["name"], g["at_station"] = name, True
        else:
            cands = [(n, v[1]) for n, v in self.prods.items() if v[1] is not None]
            if not cands:
                return g
            fp = lambda s: sorted(s)[-1] * sorted(s)[-2]
            name, size = max(cands, key=lambda c: fp(c[1]))
            Rm = np.eye(3)
            if found is not None:
                g["name"], g["at_station"] = found[0], True

        half = 0.5 * np.array(size, float)
        local = np.array([[a * half[0], b * half[1], c * half[2]]
                          for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])
        world = (Rm @ local.T).T + tp
        pc = (self._R.T @ (world - self._t).T).T
        if np.any(pc[:, 2] <= 0.02):
            return g

        lat = np.abs(pc[:, :2])
        half_px = np.array([w, h]) / 2.0 - 1
        ppm = max(w, h) / max(self.ortho_size, 1e-6)
        f = (max(w, h) / 2.0) / math.tan(self.fov / 2.0)
        if self.ortho:
            g["full_view"] = bool(np.all(lat * ppm <= half_px))
            g["need"] = 2.0 * 1.10 * float(lat.max())
            u = w / 2.0 + IMG_X_SIGN * ppm * pc[:, 0]
            v = h / 2.0 + IMG_Y_SIGN * ppm * pc[:, 1]
        else:
            ratio = lat / pc[:, 2:3]
            g["full_view"] = bool(np.all(f * ratio <= half_px))
            g["need"] = math.degrees(2.0 * math.atan(1.10 * float(ratio.max())))
            u = w / 2.0 + IMG_X_SIGN * f * pc[:, 0] / pc[:, 2]
            v = h / 2.0 + IMG_Y_SIGN * f * pc[:, 1] / pc[:, 2]

        px = np.stack([u, v], axis=1).astype(np.float32)
        hull = cv2.convexHull(px)
        poly = np.clip(hull, -2000, 2000).astype(np.int32)
        roi = np.zeros((h, w), np.uint8)
        cv2.fillConvexPoly(roi, poly, 255)
        g["roi"], g["poly"] = roi, poly
        return g


# ----------------------------- vision -----------------------------
def analyse(frame, roi, golden):
    """Find the product surface inside the projected expected area and judge it."""
    out = dict(found=False, why="")
    h, w = frame.shape[:2]
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    robot = np.zeros((h, w), np.uint8)
    for lo, hi in IGNORE_HSV:
        robot |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    robot = cv2.dilate(robot, np.ones((5, 5), np.uint8))

    roi_area = int(cv2.countNonZero(roi))
    if roi_area < 50:
        out["why"] = "Expected product area is outside the image."
        return out
    cand = (roi > 0) & (robot == 0)
    if int(cand.sum()) < MIN_VISIBLE_FRAC * roi_area:
        out["why"] = "Robot hides most of the expected product area."
        return out

    face_col = dominant(lab[cand])

    ring = ((cv2.dilate(roi, np.ones((2 * RING_PX + 1, 2 * RING_PX + 1), np.uint8)) > 0)
            & (cv2.dilate(roi, np.ones((7, 7), np.uint8)) == 0) & (robot == 0))
    if int(ring.sum()) >= 30:
        bg_col = dominant(lab[ring])
        if float(np.linalg.norm(face_col - bg_col)) < MIN_BG_SEP:
            out["why"] = "Expected area shows only background colour: product not visible."
            return out

    d_face = np.linalg.norm(lab - face_col, axis=2)
    roi_pad = cv2.dilate(roi, np.ones((2 * ROI_PAD_PX + 1, 2 * ROI_PAD_PX + 1), np.uint8)) > 0
    face = (((d_face <= FACE_TOL) & roi_pad & (robot == 0)).astype(np.uint8)) * 255
    face = cv2.morphologyEx(face, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lbl, st, _ = cv2.connectedComponentsWithStats(face, connectivity=8)
    if n < 2:
        out["why"] = "No product surface found."
        return out
    big = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[big, cv2.CC_STAT_AREA] < MIN_FACE_FRAC * roi_area:
        out["why"] = "Product surface is mostly hidden."
        return out

    face_mask = (lbl == big).astype(np.uint8) * 255
    fc, _ = cv2.findContours(face_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    fcnt = max(fc, key=cv2.contourArea)
    filled = np.zeros((h, w), np.uint8)
    cv2.drawContours(filled, [fcnt], -1, 255, -1)        # stains/holes inside the face stay inside
    if cv2.countNonZero(filled) < 60:
        out["why"] = "Product surface too small."
        return out

    x, y, bw, bh = cv2.boundingRect(fcnt)
    partial = x <= 0 or y <= 0 or x + bw >= w or y + bh >= h

    k = max(3, int(INNER_MARGIN_FRAC * min(bw, bh)))
    inner = cv2.erode(filled, np.ones((2 * k + 1, 2 * k + 1), np.uint8))
    if cv2.countNonZero(inner) < 60:
        inner = filled

    anom = ((inner > 0) & (d_face > SPOT_DIST) & (robot == 0)).astype(np.uint8) * 255
    anom = cv2.morphologyEx(anom, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n2, al, st2, _ = cv2.connectedComponentsWithStats(anom, connectivity=8)
    keep = np.zeros((h, w), np.uint8)
    spots = 0
    for i in range(1, n2):
        if st2[i, cv2.CC_STAT_AREA] >= MIN_BLOB_PX:
            keep[al == i] = 255
            spots += 1

    px_def = int(cv2.countNonZero(keep))
    pct = 100.0 * px_def / max(1, int(cv2.countNonZero(inner)))
    surface = pct >= DEFECT_AREA_PCT and px_def >= MIN_DEFECT_PX
    col_dev = float(np.linalg.norm(face_col - golden)) if golden is not None else 0.0
    colour_bad = golden is not None and col_dev > COLOUR_TOL
    face_bgr = cv2.cvtColor(np.clip(face_col, 0, 255).astype(np.uint8).reshape(1, 1, 3),
                            cv2.COLOR_LAB2BGR)[0, 0]

    out.update(found=True, bbox=(x, y, bw, bh), partial=partial,
               defect=bool(surface or colour_bad), pct=pct, px=px_def, spots=spots, anom=keep,
               col_dev=col_dev, face_lab=face_col.astype(np.float32),
               face_bgr=tuple(int(v) for v in face_bgr), face_mask=face_mask,
               reason="surface defect" if surface else ("colour off-spec" if colour_bad else ""))
    return out


# ----------------------------- KPIs -----------------------------
class Kpi:
    def __init__(self):
        self.reset()

    def reset(self):
        self.good = 0
        self.defect = 0
        self.t = []                        # simulation time of every verdict
        self.insp = []                     # inspection time of every product
        self.hist = deque(maxlen=24)
        self.last_area = None

    def add(self, verdict, t, insp, area):
        if verdict:
            self.defect += 1
        else:
            self.good += 1
        self.t.append(t)
        self.insp.append(insp)
        self.hist.append(verdict)
        self.last_area = area

    def total(self):
        return self.good + self.defect

    def defect_pct(self):
        return 100.0 * self.defect / self.total() if self.total() else 0.0

    def yield_pct(self):
        return 100.0 * self.good / self.total() if self.total() else None

    def cycle_last(self):
        return self.t[-1] - self.t[-2] if len(self.t) >= 2 else None

    def cycle_avg(self):
        return (self.t[-1] - self.t[0]) / (len(self.t) - 1) if len(self.t) >= 2 else None

    def rate_min(self):
        c = self.cycle_avg()
        return 60.0 / c if c and c > 0 else None

    def insp_avg(self):
        return sum(self.insp) / len(self.insp) if self.insp else None


def publish_kpis(sim, kpi):
    try:
        sim.setInt32Signal("KPI_Good", kpi.good)
        sim.setInt32Signal("KPI_Defect", kpi.defect)
        sim.setInt32Signal("KPI_Total", kpi.total())
        sim.setFloatSignal("KPI_DefectPct", float(kpi.defect_pct()))
        sim.setFloatSignal("KPI_RatePerMin", float(kpi.rate_min() or 0.0))
        sim.setFloatSignal("KPI_CycleTime", float(kpi.cycle_avg() or 0.0))
    except Exception:
        pass


def write_summary(path, kpi, run_id, sim_t):
    lines = [
        "SMART TEXTILE MANUFACTURING - VISION INSPECTION SUMMARY",
        "Generated        : " + datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "Run number       : %d    Simulated time: %s" % (run_id, fmt_time(sim_t)),
        "",
        "Products inspected : %d" % kpi.total(),
        "GOOD               : %d" % kpi.good,
        "DEFECT             : %d" % kpi.defect,
        "Defect rate        : %.1f %%" % kpi.defect_pct(),
        "Yield              : %s" % fv(kpi.yield_pct(), "%.1f", " %"),
        "Production rate    : %s" % fv(kpi.rate_min(), "%.2f", " products/min"),
        "Throughput         : %s" % fv(kpi.rate_min() * 60 if kpi.rate_min() else None, "%.0f", " products/hour"),
        "Average cycle time : %s" % fv(kpi.cycle_avg(), "%.2f", " s"),
        "Last cycle time    : %s" % fv(kpi.cycle_last(), "%.2f", " s"),
        "Average inspection : %s" % fv(kpi.insp_avg(), "%.2f", " s"),
        "",
        "Defect rules       : surface defect area >= %.1f %% of the product surface, or colour "
        "deviation > %.0f from the golden reference" % (DEFECT_AREA_PCT, COLOUR_TOL),
    ]
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


# ----------------------------- dashboard -----------------------------
def put(img, s, x, y, scale=0.5, col=C_TXT, th=1):
    cv2.putText(img, s, (int(x), int(y)), FONT, scale, col, th, cv2.LINE_AA)


def tsize(s, scale, th=1):
    (w, h), _ = cv2.getTextSize(s, FONT, scale, th)
    return w, h


def put_c(img, s, cx, cy, scale, col, th=1):
    w, h = tsize(s, scale, th)
    put(img, s, cx - w / 2.0, cy + h / 2.0, scale, col, th)


def fit_scale(s, scale, max_w, th=1, min_scale=0.3):
    while tsize(s, scale, th)[0] > max_w and scale > min_scale:
        scale -= 0.02
    return scale


def badge(img, x, y, w, h, text, fill):
    cv2.rectangle(img, (x, y), (x + w, y + h), fill, -1)
    put_c(img, text, x + w / 2.0, y + h / 2.0, fit_scale(text, 0.75, w - 16, 2), (255, 255, 255), 2)


def card(img, x, y, w, h, label, value, vcol):
    cv2.rectangle(img, (x, y), (x + w, y + h), C_PANEL, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), C_LINE, 1)
    put(img, label, x + 10, y + 20, 0.45, C_DIM)
    put(img, value, x + 10, y + h - 12, fit_scale(value, 0.95, w - 18, 2), vcol, 2)


def kpi_cell(img, x, y, w, h, label, value):
    cv2.rectangle(img, (x, y), (x + w, y + h), C_PANEL, -1)
    put(img, label, x + 10, y + 17, 0.40, C_DIM)
    put(img, value, x + 10, y + h - 8, fit_scale(value, 0.7, w - 20, 2), C_TXT, 2)


def wrap(text, n=58):
    lines, cur = [], ""
    for wd in text.split():
        if len(cur) + len(wd) + 1 > n:
            lines.append(cur)
            cur = wd
        else:
            cur = (cur + " " + wd).strip()
    if cur:
        lines.append(cur)
    return lines


def draw_pipeline(canvas, y, h, active):
    n, gap = len(STAGES), 14
    bw = (CANVAS_W - 16 - gap * (n - 1)) // n
    for i, name in enumerate(STAGES):
        x = 8 + i * (bw + gap)
        on = i in active
        cv2.rectangle(canvas, (x, y), (x + bw, y + h), (0, 190, 255) if on else C_PANEL, -1)
        cv2.rectangle(canvas, (x, y), (x + bw, y + h), (0, 230, 255) if on else C_LINE, 1)
        sc = fit_scale(name, 0.45, bw - 8, 2 if on else 1)
        put_c(canvas, name, x + bw / 2.0, y + h / 2.0, sc, (0, 0, 0) if on else C_DIM, 2 if on else 1)
        if i < n - 1:
            put_c(canvas, ">", x + bw + gap / 2.0, y + h / 2.0, 0.5, C_DIM, 1)


def dashboard(frame, res, poly, status, result_txt, kpi, info, sim_t, stages, packaged):
    canvas = np.full((CANVAS_H, CANVAS_W, 3), C_BG, np.uint8)

    cv2.rectangle(canvas, (0, 0), (CANVAS_W, HEADER_H), (60, 45, 20), -1)
    put(canvas, "VISION INSPECTION", 14, 34, 0.95, C_HEAD, 2)
    s1, s2 = "Smart Textile Manufacturing", "SIM TIME " + fmt_time(sim_t)
    put(canvas, s1, CANVAS_W - tsize(s1, 0.5)[0] - 14, 20, 0.5, C_TXT)
    put(canvas, s2, CANVAS_W - tsize(s2, 0.5)[0] - 14, 40, 0.5, C_DIM)
    draw_pipeline(canvas, HEADER_H + 4, 36, stages)

    # camera pane
    h, w = frame.shape[:2]
    sx, sy = CAM_PX / float(w), CAM_PX / float(h)
    cam = cv2.resize(frame, (CAM_PX, CAM_PX), interpolation=cv2.INTER_NEAREST)
    if poly is not None:
        pts = (poly.reshape(-1, 2).astype(np.float32) * np.array([sx, sy], np.float32)).astype(np.int32)
        cv2.polylines(cam, [pts], True, (0, 220, 255), 2)
    if res.get("found"):
        a = cv2.resize(res["anom"], (CAM_PX, CAM_PX), interpolation=cv2.INTER_NEAREST)
        cam[a > 0] = (0, 0, 255)
        x, y, bw, bh = res["bbox"]
        col = C_BAD if res["defect"] else C_GOOD
        p1, p2 = (int(x * sx), int(y * sy)), (int((x + bw) * sx), int((y + bh) * sy))
        cv2.rectangle(cam, p1, p2, col, 3)
        tag = "DEFECT" if res["defect"] else "GOOD"
        tw, th = tsize(tag, 0.6, 2)
        ty0 = max(p1[1] - th - 14, 0)
        cv2.rectangle(cam, (p1[0], ty0), (p1[0] + tw + 12, ty0 + th + 10), col, -1)
        put(cam, tag, p1[0] + 6, ty0 + th + 3, 0.6, (255, 255, 255), 2)
    cv2.rectangle(cam, (0, CAM_PX - 24), (CAM_PX, CAM_PX), (0, 0, 0), -1)
    put(cam, "Inspection_Camera  %dx%d" % (w, h), 8, CAM_PX - 8, 0.45, C_DIM)
    y0 = HEADER_H + STRIP_H
    canvas[y0:y0 + CAM_PX, 0:CAM_PX] = cam

    # status badges + counters
    px = CAM_PX + 16
    badge(canvas, px, y0 + 8, 258, 44, "PRODUCT: " + status, STATUS_FILL.get(status, (70, 70, 70)))
    badge(canvas, px + 270, y0 + 8, 258, 44, "RESULT: " + result_txt, RESULT_FILL.get(result_txt, (70, 70, 70)))
    cards = [("GOOD", str(kpi.good), C_GOOD), ("DEFECT", str(kpi.defect), C_BAD),
             ("TOTAL", str(kpi.total()), C_TXT),
             ("DEFECT %", "%.1f%%" % kpi.defect_pct(), C_WARN if kpi.defect else C_TXT)]
    for i, (lab, val, col) in enumerate(cards):
        card(canvas, px + i * 134, y0 + 62, 126, 70, lab, val, col)

    # KPIs
    put(canvas, "PRODUCTION KPIs", px, y0 + 154, 0.55, C_HEAD, 1)
    cv2.line(canvas, (px + 160, y0 + 149), (px + 528, y0 + 149), C_LINE, 1)
    rate = kpi.rate_min()
    cells = [("PRODUCTION RATE", fv(rate, "%.1f", " /min")),
             ("THROUGHPUT", fv(rate * 60 if rate else None, "%.0f", " /hour")),
             ("LAST CYCLE TIME", fv(kpi.cycle_last(), "%.1f", " s")),
             ("AVG CYCLE TIME", fv(kpi.cycle_avg(), "%.1f", " s")),
             ("INSPECTION TIME", fv(kpi.insp_avg(), "%.2f", " s")),
             ("YIELD (GOOD %)", fv(kpi.yield_pct(), "%.1f", " %")),
             ("PACKAGED", "--" if packaged is None else str(packaged)),
             ("LAST DEFECT AREA", fv(kpi.last_area, "%.1f", " %"))]
    for i, (lab, val) in enumerate(cells):
        kpi_cell(canvas, px + (i % 2) * 270, y0 + 168 + (i // 2) * 48, 258, 44, lab, val)

    # history strip
    put(canvas, "LAST 24 RESULTS   (green = good, red = defect)", px, y0 + 376, 0.45, C_DIM)
    hist = list(kpi.hist)
    for i in range(24):
        x = px + i * 22
        if i < len(hist):
            cv2.rectangle(canvas, (x, y0 + 384), (x + 20, y0 + 404), C_BAD if hist[i] else C_GOOD, -1)
        else:
            cv2.rectangle(canvas, (x, y0 + 384), (x + 20, y0 + 404), C_LINE, 1)

    # info lines
    lines = []
    for t in info:
        lines += wrap(t)
    for i, line in enumerate(lines[:5]):
        put(canvas, line, px, y0 + 424 + i * 19, 0.42, C_DIM)
    return canvas


def camera_msg(g):
    if g.get("full_view") is None:
        return ""
    if g["full_view"]:
        return "CAMERA OK: whole product in view (%s)." % g["view_txt"]
    if g["ortho"]:
        return ("SET Inspection_Camera ORTHO SIZE = %.2f m (now %.2f m): stop the simulation and "
                "run camera_setup.py." % (math.ceil(g["need"] * 100) / 100.0, g["cur"]))
    return "SET Inspection_Camera PERSPECTIVE ANGLE = %d deg (now %s)." % (
        math.ceil(g["need"]), g["view_txt"])


def new_episode():
    return dict(streak=0, last_v=None, stable=None, counted=False, gone=0, samples=[], t0=None)


# ----------------------------- main -----------------------------
def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    print("Connecting to CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.getObject("sim")
    scene = Scene(sim)
    gate_ok = scene.trig is not None and len(scene.prods) > 0
    print("Connected. Station knowledge:", "ON" if gate_ok else "OFF (vision only)")
    print("Press Play in CoppeliaSim.  q = quit, s = snapshot, d = debug mask, r = reset counters.")

    csv_path = os.path.join(SAVE_DIR, "results.csv")
    new_csv = not os.path.exists(csv_path)
    csv_file = open(csv_path, "a", newline="")
    writer = csv.writer(csv_file)
    if new_csv:
        writer.writerow(["wall_time", "run", "sim_time_s", "verdict", "defect_area_pct", "spots",
                         "colour_dev", "reason", "cycle_time_s", "inspection_time_s"])

    golden = None
    if GOOD_REF_BGR is not None:
        golden = cv2.cvtColor(np.uint8([[GOOD_REF_BGR]]), cv2.COLOR_BGR2LAB)[0, 0].astype(np.float32)
    learned = golden is not None

    kpi = Kpi()
    run_id, sim_t = 0, 0.0
    ep = new_episode()
    prev_present, sig_ok, was_stopped = False, True, True
    held, hold_until = None, 0.0
    any_canvas, raw = None, None
    show_dbg, last_err, last_cam_msg = False, "", ""

    try:
        while True:
            try:
                if sim.getSimulationState() == sim.simulation_stopped:
                    c = np.full((CANVAS_H, CANVAS_W, 3), C_BG, np.uint8)
                    put_c(c, "SIMULATION STOPPED - press Play in CoppeliaSim",
                          CANVAS_W / 2.0, CANVAS_H / 2.0, 0.9, C_WARN, 2)
                    cv2.imshow(WINDOW, c)
                    was_stopped = True
                    if cv2.waitKey(200) & 0xFF == ord("q"):
                        break
                    continue
                if was_stopped:
                    was_stopped = False
                    run_id += 1
                    kpi.reset()
                    ep, prev_present, held, hold_until = new_episode(), False, None, 0.0
                    print("--- run %d started: counters and KPIs reset ---" % run_id)

                sim_t = sim.getSimulationTime()
                frame = grab(sim, scene.cam)
                raw = frame.copy()
                h, w = frame.shape[:2]

                active = robot_state = packaged = None
                if sig_ok:
                    try:
                        v = sim.getInt32Signal(ACTIVE_SIGNAL)
                        active = None if v is None else int(v)
                        v = sim.getInt32Signal(STATE_SIGNAL)
                        robot_state = None if v is None else int(v)
                        v = sim.getInt32Signal(PACKAGED_SIGNAL)
                        packaged = None if v is None else int(v)
                    except Exception:
                        sig_ok = False

                geo = scene.geometry(w, h)
                if active is not None:
                    present = (active == 1)
                elif gate_ok:
                    present = geo["at_station"]
                else:
                    present = None
                gated = present is not None

                if gated and present and not prev_present:
                    ep, hold_until = new_episode(), 0.0
                    ep["t0"] = sim_t
                if gated:
                    prev_present = present

                res, hint, status = dict(found=False), "", "WAITING"
                dbg_img, poly = None, geo["poly"]
                if gated and not present:
                    hint = "No product at the inspection station."
                elif geo["roi"] is None:
                    status = "NOT VISIBLE" if gated else "WAITING"
                    hint = "Station is outside the camera view, or scene objects are missing."
                else:
                    res = analyse(frame, geo["roi"], golden)
                    if res["found"]:
                        status = "DETECTED"
                        dbg_img = cv2.cvtColor(res["face_mask"], cv2.COLOR_GRAY2BGR)
                        if res["partial"]:
                            hint = "Product is cut by the frame edge: inspecting the visible part."
                    else:
                        status = "NOT VISIBLE" if gated else "WAITING"
                        hint = res["why"]

                name = geo["name"]
                if res["found"] and TEACH_GOLDEN and gate_ok and not learned and name == KNOWN_GOOD:
                    ep["samples"].append(res["face_lab"])
                    if len(ep["samples"]) >= 8:
                        golden = np.median(np.array(ep["samples"]), axis=0).astype(np.float32)
                        learned = True
                        print("Reference colour learned from", KNOWN_GOOD)
                calibrating = bool(res["found"] and TEACH_GOLDEN and gate_ok
                                   and not learned and name != KNOWN_GOOD)

                just_counted = False
                if res["found"] and not calibrating:
                    ep["gone"] = 0
                    if not ep["counted"]:            # verdict is locked once it has been counted
                        v = 1 if res["defect"] else 0
                        ep["streak"] = ep["streak"] + 1 if v == ep["last_v"] else 1
                        ep["last_v"] = v
                        if ep["streak"] >= STABLE_FRAMES:
                            ep["stable"] = v
                            ep["counted"], just_counted = True, True
                            insp = (sim_t - ep["t0"]) if ep["t0"] is not None else 0.0
                            kpi.add(v, sim_t, insp, res["pct"])
                            hold_until = time.time() + HOLD_SEC
                            if PUBLISH_KPI_SIGNALS:
                                publish_kpis(sim, kpi)
                else:
                    ep["streak"], ep["last_v"] = 0, None
                    if not res["found"]:
                        ep["gone"] += 1
                        if not gated and ep["gone"] >= 8:
                            ep = new_episode()

                out = ep["stable"] if (ep["stable"] is not None and present is not False) else -1
                if PUBLISH_SIGNAL:
                    sim.setFloatSignal(RESULT_SIGNAL, float(out))

                if ep["stable"] is not None and present is not False:
                    result_txt = "GOOD" if ep["stable"] == 0 else "DEFECT"
                elif calibrating:
                    result_txt = "CALIBRATING"
                elif res["found"]:
                    result_txt = "ANALYSING"
                else:
                    result_txt = "WAITING"

                info = []
                if res["found"]:
                    info.append("defect spots: %d   area %.1f%% (limit %.1f%%)" %
                                (res["spots"], res["pct"], DEFECT_AREA_PCT))
                    info.append("face colour BGR %s" % (res["face_bgr"],))
                    if golden is not None:
                        info.append("colour deviation %.0f (limit %.0f)" % (res["col_dev"], COLOUR_TOL))
                    if ep["stable"] == 1 and res["reason"]:
                        info.append("reason: " + res["reason"])
                if calibrating:
                    info.append("CALIBRATING: waiting for a known-good Product_A to learn the "
                                "reference colour.")
                cm = camera_msg(geo)
                if cm:
                    info.append(cm)
                if hint:
                    info.append(hint)

                if robot_state is not None:
                    stages = STATE_TO_STAGES.get(robot_state, [])
                else:
                    stages = [4, 5] if present else []

                view = dict(frame=frame, res=res, poly=poly, status=status,
                            result_txt=result_txt, info=info)
                if ep["stable"] is not None and present is not False:
                    held = view
                if gated and not present and held is not None and time.time() < hold_until:
                    view = held
                canvas = dashboard(view["frame"], view["res"], view["poly"], view["status"],
                                   view["result_txt"], kpi, view["info"], sim_t, stages, packaged)
                any_canvas = canvas
                cv2.imshow(WINDOW, canvas)

                if cm and cm != last_cam_msg:
                    print(cm)
                    last_cam_msg = cm

                if just_counted:
                    tag = "DEFECT" if res["defect"] else "GOOD"
                    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(os.path.join(SAVE_DIR, "%s_%s.png" % (stamp, tag)), canvas)
                    cyc = kpi.cycle_last()
                    writer.writerow([stamp, run_id, round(sim_t, 2), tag, round(res["pct"], 2),
                                     res["spots"], round(res["col_dev"], 1), res["reason"],
                                     "" if cyc is None else round(cyc, 2), round(kpi.insp[-1], 2)])
                    csv_file.flush()
                    print("[%s] %s  spots %d  area %.1f%%  | GOOD %d  DEFECT %d  rate %s  cycle %s" %
                          (stamp, tag, res["spots"], res["pct"], kpi.good, kpi.defect,
                           fv(kpi.rate_min(), "%.1f/min"), fv(kpi.cycle_avg(), "%.1fs")))

                if show_dbg and dbg_img is not None:
                    cv2.imshow("debug mask", cv2.resize(dbg_img, None, fx=2, fy=2,
                                                        interpolation=cv2.INTER_NEAREST))
                last_err = ""
            except Exception as exc:
                if str(exc) != last_err:
                    print("Frame error:", exc)
                    last_err = str(exc)
                time.sleep(0.2)

            key = cv2.waitKey(15) & 0xFF
            if key == ord("q"):
                break
            if key == ord("r"):
                kpi.reset()
                ep, held = new_episode(), None
                print("Counters reset.")
            if key == ord("s") and raw is not None:
                name_png = os.path.join(SAVE_DIR, "snap_%s.png" % datetime.now().strftime("%H%M%S"))
                cv2.imwrite(name_png, raw)
                print("Saved", name_png)
            if key == ord("d"):
                show_dbg = not show_dbg
                if not show_dbg:
                    cv2.destroyWindow("debug mask")
    finally:
        if PUBLISH_SIGNAL:
            try:
                sim.clearFloatSignal(RESULT_SIGNAL)
            except Exception:
                pass
        try:
            if kpi.total() > 0:
                write_summary(os.path.join(SAVE_DIR, "kpi_summary.txt"), kpi, run_id, sim_t)
                print("Saved", os.path.join(SAVE_DIR, "kpi_summary.txt"))
            if any_canvas is not None:
                cv2.imwrite(os.path.join(SAVE_DIR, "final_dashboard.png"), any_canvas)
        except Exception as exc:
            print("Could not write summary:", exc)
        csv_file.close()
        cv2.destroyAllWindows()
        print("Inspection stopped.")


if __name__ == "__main__":
    main()