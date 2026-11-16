#!/usr/bin/env python3
"""Parametric keycaps for 12x12 mm tactile switches with round actuators.

Generates two STLs in the same coordinate frame:
  keycaps_body.stl   - the six cap bodies
  keycaps_labels.stl - raised labels (load together as one multi-part object
                       and give the labels a second colour)

Edit the PARAMETERS block and re-run:  python3 keycaps.py
"""
import math, struct
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# ---------------- PARAMETERS (mm) ----------------
STEM_TOP_D    = 6.0    # actuator diameter at its tip (measured)
STEM_BASE_D   = 6.8    # actuator diameter at its base (measured)
STEM_LEN      = 8.8    # length of the tapered actuator above the switch body (measured)
FIT_CLEAR     = -0.05  # added to the socket diameter; FDM holes print slightly small,
                       # so ~0 grips. Tighter: -0.1. Looser: +0.05
SOCKET_DEPTH  = 4.0    # how far the cap slides down over the actuator
CAP_HEIGHT    = 9.0    # total cap height excl. label (adds CAP_HEIGHT-SOCKET_DEPTH on top of actuator)
CAP_SIZE      = 11.5   # width of square caps / diameter of round zoom caps
CORNER_R      = 1.6    # corner radius of square caps
TOP_CHAMFER   = 0.6
FLANGE_SIZE   = 13.5   # retaining lip under the lid (set = CAP_SIZE to disable)
FLANGE_H      = 1.0
LABEL_H       = 0.6    # raised label height
DIGIT_H       = 6.0    # digit cap height
SYMBOL_W      = 6.5    # width of + / - bars
SYMBOL_T      = 1.5    # stroke thickness of + / -
TRI_INRADIUS  = 4.8    # zoom triangles: centre-to-edge distance (sets the wall round the socket)
TRI_CORNER_R  = 2.0    # corner rounding of the triangles
TRI_SYMBOL_W  = 4.4    # +/- size on the triangles
TRI_SYMBOL_T  = 1.3
FONT          = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
PITCH         = 19.0   # spacing on the print plate
N             = 240    # angular resolution
PX            = 0.04   # label raster resolution
# --------------------------------------------------

CAPS = [  # (label, shape) laid out to match the board: zoom column, then 1-4
    ("+", "tri_up"), ("1", "square"), ("2", "square"),
    ("-", "tri_down"), ("3", "square"), ("4", "square"),
]

tris = {"body": [], "labels": []}

def add_tri(key, a, b, c, ref):
    a, b, c = map(np.asarray, (a, b, c))
    n = np.cross(b - a, c - a)
    if np.dot(n, ref(a, b, c)) < 0:
        b, c = c, b
    tris[key].append((a, b, c))

def rbox_radius(theta, half, r):
    """Distance from centre to rounded-square boundary along angle theta."""
    d = np.array([math.cos(theta), math.sin(theta)])
    def sdf(t):
        p = np.abs(d * t) - (half - r)
        return np.linalg.norm(np.maximum(p, 0)) + min(max(p[0], p[1]), 0) - r
    lo, hi = 0.0, half * 2
    for _ in range(50):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if sdf(mid) < 0 else (lo, mid)
    return lo

ANG = np.linspace(0, 2 * math.pi, N, endpoint=False)

def tri_radius(theta, up, r):
    """Ray length from centroid to a rounded equilateral triangle (core offset by r)."""
    core_in = TRI_INRADIUS - TRI_CORNER_R
    R = 2 * core_in
    base = math.pi / 2 if up else -math.pi / 2
    V = [np.array([R * math.cos(base + k * 2 * math.pi / 3), R * math.sin(base + k * 2 * math.pi / 3)]) for k in range(3)]
    def dist(p):
        dmin = min(np.linalg.norm(p - (a + np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0, 1) * (b - a)))
                   for a, b in zip(V, V[1:] + V[:1]))
        inside = all((b - a)[0] * (p - a)[1] - (b - a)[1] * (p - a)[0] >= 0 for a, b in zip(V, V[1:] + V[:1]))
        return -dmin if inside else dmin
    d = np.array([math.cos(theta), math.sin(theta)])
    lo, hi = 0.0, 4 * TRI_INRADIUS + r
    for _ in range(50):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if dist(d * mid) - r < 0 else (lo, mid)
    return lo

def loop(shape, size, cx, cy):
    half = size / 2
    if shape.startswith("tri"):
        r = TRI_CORNER_R + (size - CAP_SIZE) / 2          # offset the outline like the other shapes
        rs = np.array([tri_radius(t, shape == "tri_up", r) for t in ANG])
    elif shape == "round":
        rs = np.full(N, half)
    else:
        r = min(CORNER_R * size / CAP_SIZE, half)
        rs = np.array([rbox_radius(t, half, r) for t in ANG])
    return np.stack([cx + rs * np.cos(ANG), cy + rs * np.sin(ANG)], 1)

def strip(key, A, za, B, zb, ref):
    for i in range(N):
        j = (i + 1) % N
        p = [(*A[i], za), (*A[j], za), (*B[j], zb), (*B[i], zb)]
        add_tri(key, p[0], p[1], p[2], ref)
        add_tri(key, p[0], p[2], p[3], ref)

def fan(key, L, z, c, ref):
    for i in range(N):
        j = (i + 1) % N
        add_tri(key, (*c, z), (*L[i], z), (*L[j], z), ref)

def cap_body(shape, cx, cy):
    c = np.array([cx, cy])
    up = lambda *a: np.array([0, 0, 1.0])
    down = lambda *a: np.array([0, 0, -1.0])
    def out(a, b, cc):
        m = (a + b + cc) / 3; v = np.array([m[0] - cx, m[1] - cy, 0]); return v
    def inw(a, b, cc):
        return -out(a, b, cc)
    def out_up(a, b, cc):
        return out(a, b, cc) / 10 + np.array([0, 0, 1.0])
    F = loop(shape, max(FLANGE_SIZE, CAP_SIZE), cx, cy)
    O = loop(shape, CAP_SIZE, cx, cy)
    Oi = loop(shape, CAP_SIZE - 2 * TOP_CHAMFER, cx, cy)
    # tapered socket matching the actuator: socket ceiling sits on the actuator tip
    d_top = STEM_TOP_D + FIT_CLEAR
    d_mouth = STEM_TOP_D + (STEM_BASE_D - STEM_TOP_D) * min(SOCKET_DEPTH / STEM_LEN, 1) + FIT_CLEAR
    S = np.stack([cx + d_mouth / 2 * np.cos(ANG), cy + d_mouth / 2 * np.sin(ANG)], 1)
    St = np.stack([cx + d_top / 2 * np.cos(ANG), cy + d_top / 2 * np.sin(ANG)], 1)
    strip("body", S, 0, F, 0, down)                       # bottom ring
    strip("body", F, 0, F, FLANGE_H, out)                 # flange edge
    strip("body", F, FLANGE_H, O, FLANGE_H, up)           # flange top
    strip("body", O, FLANGE_H, O, CAP_HEIGHT - TOP_CHAMFER, out)
    strip("body", O, CAP_HEIGHT - TOP_CHAMFER, Oi, CAP_HEIGHT, out_up)
    fan("body", Oi, CAP_HEIGHT, c, up)                    # top
    strip("body", S, 0, St, SOCKET_DEPTH, inw)            # tapered socket wall
    fan("body", St, SOCKET_DEPTH, c, down)                # socket ceiling

def label_mask(text, shape="square"):
    s = int(round(CAP_SIZE / PX))
    img = Image.new("L", (s, s), 0)
    d = ImageDraw.Draw(img)
    m = s / 2
    if text in "+-":
        sw, st = (TRI_SYMBOL_W, TRI_SYMBOL_T) if shape.startswith("tri") else (SYMBOL_W, SYMBOL_T)
        w, t = sw / PX / 2, st / PX / 2
        d.rectangle([m - w, m - t, m + w, m + t], fill=255)
        if text == "+":
            d.rectangle([m - t, m - w, m + t, m + w], fill=255)
    else:
        font = ImageFont.truetype(FONT, 100)
        bb = font.getbbox(text)
        size = int(100 * (DIGIT_H / PX) / (bb[3] - bb[1]))
        font = ImageFont.truetype(FONT, size)
        bb = font.getbbox(text)
        d.text((m - (bb[0] + bb[2]) / 2, m - (bb[1] + bb[3]) / 2), text, font=font, fill=255)
    return np.array(img) > 127

def label_mesh(text, cx, cy, shape):
    M = label_mask(text, shape)
    h, w = M.shape
    z0, z1 = CAP_HEIGHT - 0.01, CAP_HEIGHT + LABEL_H   # tiny overlap so parts fuse
    x0, y0 = cx - CAP_SIZE / 2, cy + CAP_SIZE / 2
    X = lambda i: x0 + i * PX
    Y = lambda j: y0 - j * PX                           # image rows go down
    up = lambda *a: np.array([0, 0, 1.0]); down = lambda *a: np.array([0, 0, -1.0])
    P = np.pad(M, 1)
    ys, xs = np.nonzero(M)
    for j, i in zip(ys, xs):
        a, b, c_, d_ = (X(i), Y(j)), (X(i + 1), Y(j)), (X(i + 1), Y(j + 1)), (X(i), Y(j + 1))
        for z, ref in ((z1, up), (z0, down)):
            add_tri("labels", (*a, z), (*b, z), (*c_, z), ref)
            add_tri("labels", (*a, z), (*c_, z), (*d_, z), ref)
        for (di, dj), (p, q) in (((0, -1), (a, b)), ((1, 0), (b, c_)),
                                 ((0, 1), (c_, d_)), ((-1, 0), (d_, a))):
            if not P[j + 1 + dj, i + 1 + di]:
                n = np.array([di, -dj, 0.0])
                ref = lambda *x, n=n: n
                add_tri("labels", (*p, z0), (*q, z0), (*q, z1), ref)
                add_tri("labels", (*p, z0), (*q, z1), (*p, z1), ref)

def write_stl(path, T):
    with open(path, "wb") as f:
        f.write(b"keycaps".ljust(80, b" ")); f.write(struct.pack("<I", len(T)))
        for a, b, c in T:
            n = np.cross(b - a, c - a); l = np.linalg.norm(n); n = n / l if l else n
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))

for k, (text, shape) in enumerate(CAPS):
    cx, cy = (k % 3) * PITCH, -(k // 3) * PITCH
    cap_body(shape, cx, cy)
    label_mesh(text, cx, cy, shape)

write_stl("keycaps_body.stl", tris["body"])
write_stl("keycaps_labels.stl", tris["labels"])
print("body tris", len(tris["body"]), "label tris", len(tris["labels"]))
