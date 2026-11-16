#!/usr/bin/env python3
"""PTZ controller enclosure: box with standoffs + USB side slot, and a press-fit lid.

Everything is laid out on the 2.54 mm perfboard grid. Origin = front-left
mounting hole. i = holes to the right, j = holes towards the back.

Outputs: box.stl, lid.stl (lid is already flipped top-face-down for printing),
         board_layout.png (drilling / placement drawing)
Needs only numpy + matplotlib.  python3 enclosure.py
"""
import math, struct
import numpy as np
from matplotlib.path import Path
from matplotlib.tri import Triangulation

G = 2.54
# ---------------- BOARD LAYOUT (hole coordinates) ----------------
MOUNT = [(0, 0), (40, 0), (0, 18), (40, 18)]          # drill these out to 3 mm
BOARD_MARGIN = 1.5                                     # holes beyond mounting holes (cut between rows)
JOY = (7, 8)                                           # joystick shaft centre (hole coords)
# KY-023 module, header facing the front edge. Offsets in mm from the shaft centre
# (measured: holes 19 wide x 26 deep, shaft 8 mm from left holes, 14 mm from top holes)
JOY_HOLE_W, JOY_HOLE_H = 19.0, 26.0
JOY_SHAFT_FROM_LEFT, JOY_SHAFT_FROM_TOP = 8.0, 14.0
JOY_MOUNT_D = 3.0
JOY_DOME_D = 26.0
# switch centres: on a hole row, halfway between columns (legs at i+/-2.5, j+/-1)
SWITCHES = [  # (name, i, j, lid-hole shape)
    ("Z+", 17.5, 14, "tri_up"),  ("1", 27.5, 14, "square"), ("2", 37.5, 14, "square"),
    ("Z-", 17.5, 4,  "tri_down"), ("3", 27.5, 4,  "square"), ("4", 37.5, 4,  "square"),
]
PROMICRO = dict(x0=-BOARD_MARGIN * G, len=33.0, w=18.0, cy=9 * G)   # under board, USB at left edge

# ---------------- HEIGHTS (mm) ----------------
STEM_TOP     = 12.0   # top of switch stems above perfboard top surface  <-- MEASURE
CAP_RISE     = 5.0    # keycap CAP_HEIGHT - SOCKET_DEPTH (keycaps.py default 9 - 4)
PROTRUDE     = 2.5    # how far caps stand proud of the lid
PCB_T        = 1.6
FLOOR        = 2.0
STANDOFF_H   = 10.0   # room for the Pro Micro + wiring under the board
STANDOFF_D   = 6.0
PILOT_D      = 2.5    # M3 self-tapping screw (use 2.2 for M2.5)
MID_STANDOFFS = [(22, 9), (32, 9)]   # extra M3 standoffs between the switches (hole coords)
LEDGE        = 1.5    # how far the edge ledge reaches under the board
LEDGE_GAP_W  = 20.0   # gap in the left ledge for the Pro Micro / USB plug
LID_POST_D   = 5.0    # lid posts at the corner holes: clamp the board, take the lid screws
LID_SCREW_D  = 3.4    # clearance hole for M3 through lid + post
POST_GAP     = 0.2    # post stops this far above the board so the lid always seats

# ---------------- BOX / LID ----------------
WALL         = 2.0
GAP          = 0.75   # board edge to inner wall
CORNER_R     = 3.0    # outer corner radius
LID_T        = 2.0
LIP_H        = 4.0    # locating lip under the lid
LIP_T        = 1.2
FIT          = 0.2    # lip-to-wall clearance (raise if too tight)
HOLE_CLEAR   = 0.4    # keycap-to-lid-hole clearance per side
JOY_OPEN_D   = 26.5   # round lid opening for the joystick
USB_SLOT_W   = 12.0
USB_OPEN_H   = 8.0    # height of the opening left for the cable under the lid tab
# keycap outline params (match keycaps.py)
CAP_SIZE, CAP_CORNER, TRI_IN, TRI_R = 11.5, 1.6, 4.8, 2.0
STEP = 0.4            # boundary sampling (mm)

# ---------------- derived ----------------
def joy_mount_holes():
    sx, sy = JOY[0] * G, JOY[1] * G
    xl = sx - JOY_SHAFT_FROM_LEFT; yt = sy + JOY_SHAFT_FROM_TOP
    return [(xl, yt), (xl + JOY_HOLE_W, yt), (xl, yt - JOY_HOLE_H), (xl + JOY_HOLE_W, yt - JOY_HOLE_H)]
bx0 = -BOARD_MARGIN * G; by0 = -BOARD_MARGIN * G
bx1 = (max(m[0] for m in MOUNT) + BOARD_MARGIN) * G
by1 = (max(m[1] for m in MOUNT) + BOARD_MARGIN) * G
ix0, iy0, ix1, iy1 = bx0 - GAP, by0 - GAP, bx1 + GAP, by1 + GAP      # inner walls
ox0, oy0, ox1, oy1 = ix0 - WALL, iy0 - WALL, ix1 + WALL, iy1 + WALL  # outer
Z_PCB_BOT = FLOOR + STANDOFF_H
Z_PCB_TOP = Z_PCB_BOT + PCB_T
Z_LID_TOP = Z_PCB_TOP + STEM_TOP + CAP_RISE - PROTRUDE
Z_WALL    = Z_LID_TOP - LID_T
SLOT_BOT  = FLOOR + 1.0
TAB_BOT   = SLOT_BOT + USB_OPEN_H

# ================= geometry helpers =================
def offset_poly(V, r, step=STEP):
    """Convex CCW polygon V offset outward by r, sampled as a CCW loop."""
    V = [np.asarray(v, float) for v in V]
    n = len(V); pts = []
    for k in range(n):
        a, b, c = V[k - 1], V[k], V[(k + 1) % n]
        e1, e2 = b - a, c - b
        n1 = np.array([e1[1], -e1[0]]) / np.linalg.norm(e1)
        n2 = np.array([e2[1], -e2[0]]) / np.linalg.norm(e2)
        t1, t2 = math.atan2(n1[1], n1[0]), math.atan2(n2[1], n2[0])
        if t2 < t1: t2 += 2 * math.pi
        m = max(2, int(math.ceil(r * (t2 - t1) / step)) + 1) if r > 0 else 1
        for t in np.linspace(t1, t2, m):
            pts.append(b + r * np.array([math.cos(t), math.sin(t)]))
        # straight edge to next corner
        p0 = b + r * n2; p1 = c + r * n2
        L = np.linalg.norm(p1 - p0); m = int(L // step)
        for s in range(1, m):
            pts.append(p0 + (p1 - p0) * s / m)
    return dedupe(pts)

def dedupe(pts):
    out = []
    for p in pts:
        if not out or np.linalg.norm(p - out[-1]) > 1e-6: out.append(p)
    if np.linalg.norm(out[0] - out[-1]) < 1e-6: out.pop()
    return np.array(out)

def rrect(x0, y0, x1, y1, r):
    return offset_poly([(x0 + r, y0 + r), (x1 - r, y0 + r), (x1 - r, y1 - r), (x0 + r, y1 - r)], r)

def circle(cx, cy, d):
    n = max(24, int(math.pi * d / STEP))
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.stack([cx + d / 2 * np.cos(t), cy + d / 2 * np.sin(t)], 1)

def tri(cx, cy, up, grow):
    core = TRI_IN - TRI_R; R = 2 * core
    base = math.pi / 2 if up else -math.pi / 2
    V = [(cx + R * math.cos(base + k * 2 * math.pi / 3), cy + R * math.sin(base + k * 2 * math.pi / 3)) for k in range(3)]
    return offset_poly(V, TRI_R + grow)

def area(L):
    x, y = L[:, 0], L[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)

def ccw(L, want=True):
    return L if (area(L) > 0) == want else L[::-1]

def resample(L, step=STEP):
    out = []
    for k in range(len(L)):
        p, q = L[k], L[(k + 1) % len(L)]
        m = max(1, int(math.ceil(np.linalg.norm(q - p) / step - 0.5)))
        out += [p + (q - p) * s / m for s in range(m)]
    return np.array(out)

TRIS = []
def tri_add(a, b, c):
    TRIS.append((np.asarray(a, float), np.asarray(b, float), np.asarray(c, float)))

def extrude(outer, holes, z0, z1):
    """Closed prism of polygon-with-holes from z0 to z1 (watertight)."""
    outer = ccw(resample(np.asarray(outer))); holes = [ccw(resample(np.asarray(h)), False) for h in holes]
    loops = [outer] + holes
    B = np.vstack(loops)
    # interior Steiner points keep Delaunay edges on the boundary and triangles tidy
    xs = np.arange(outer[:, 0].min(), outer[:, 0].max(), 1.6)
    ys = np.arange(outer[:, 1].min(), outer[:, 1].max(), 1.6)
    gx, gy = np.meshgrid(xs, ys); S = np.stack([gx.ravel(), gy.ravel()], 1)
    S = S[Path(outer).contains_points(S)]
    for h in holes: S = S[~Path(h).contains_points(S)]
    if len(S):
        d = np.min(np.hypot(S[:, None, 0] - B[None, :, 0], S[:, None, 1] - B[None, :, 1]), 1)
        S = S[d > 1.0]
    P = np.vstack([B, S]) if len(S) else B
    T = Triangulation(P[:, 0], P[:, 1]).triangles
    C = P[T].mean(1)
    keep = Path(outer).contains_points(C)
    for h in holes: keep &= ~Path(h).contains_points(C)
    for t in T[keep]:
        a, b, c = P[t]
        if (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]) < 0: b, c = c, b
        tri_add((*a, z1), (*b, z1), (*c, z1))
        tri_add((*a, z0), (*c, z0), (*b, z0))
    for L in loops:
        for k in range(len(L)):
            p, q = L[k], L[(k + 1) % len(L)]
            tri_add((*p, z0), (*q, z0), (*q, z1))
            tri_add((*p, z0), (*q, z1), (*p, z1))

def write_stl(path, T):
    with open(path, "wb") as f:
        f.write(b"ptz enclosure".ljust(80, b" ")); f.write(struct.pack("<I", len(T)))
        for a, b, c in T:
            n = np.cross(b - a, c - a); l = np.linalg.norm(n); n = n / l if l else n
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))

def wall_with_slot(z0, z1):
    """Wall ring with a gap in the left wall for the USB cable, as one polygon."""
    cy = PROMICRO["cy"]; h = USB_SLOT_W / 2
    def cut(L, x_edge):
        L = ccw(L)
        on = np.isclose(L[:, 0], x_edge) & (np.abs(L[:, 1] - cy) <= h)
        L = L[~on]
        # rotate so we start just below the gap on the left wall, going down (CCW)
        k = np.argmin(np.where(np.isclose(L[:, 0], x_edge) & (L[:, 1] < cy), np.abs(L[:, 1] - cy), 1e9))
        return np.roll(L, -k, 0)
    O = cut(rrect(ox0, oy0, ox1, oy1, CORNER_R), ox0)
    I = cut(rrect(ix0, iy0, ix1, iy1, max(CORNER_R - WALL, 0.5)), ix0)
    poly = np.vstack([[[ox0, cy - h]], O, [[ox0, cy + h]], [[ix0, cy + h]], I[::-1], [[ix0, cy - h]]])
    extrude(dedupe(list(poly)), [], z0, z1)

def mm(i, j): return i * G, j * G

# ================= BOX =================
TRIS.clear()
outer_box = rrect(ox0, oy0, ox1, oy1, CORNER_R)
inner_box = rrect(ix0, iy0, ix1, iy1, max(CORNER_R - WALL, 0.5))
extrude(outer_box, [], 0, FLOOR)                                   # floor
extrude(outer_box, [inner_box], FLOOR - 0.5, SLOT_BOT)             # wall below slot
wall_with_slot(SLOT_BOT - 0.5, Z_WALL)                             # wall with USB slot
for (i, j) in MOUNT + MID_STANDOFFS:                               # standoffs
    x, y = mm(i, j)
    extrude(circle(x, y, STANDOFF_D), [circle(x, y, PILOT_D)], FLOOR - 0.5, Z_PCB_BOT)
# ledge round the inside of the wall that the board edges rest on (gap at the Pro Micro)
e = GAP + LEDGE; cyp = PROMICRO["cy"]; g = LEDGE_GAP_W / 2
for (x0, y0, x1, y1) in [(ix0 - 0.5, iy0 - 0.5, ix1 + 0.5, iy0 + e),            # front
                         (ix0 - 0.5, iy1 - e, ix1 + 0.5, iy1 + 0.5),            # back
                         (ix1 - e, iy0, ix1 + 0.4, iy1),                        # right
                         (ix0 - 0.4, iy0, ix0 + e, cyp - g),                    # left, front of gap
                         (ix0 - 0.4, cyp + g, ix0 + e, iy1)]:                   # left, behind gap
    extrude(np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]), [], FLOOR - 0.5, Z_PCB_BOT)
write_stl("box.stl", TRIS); box_tris = list(TRIS)

# ================= LID (modelled upright, then flipped) =================
TRIS.clear()
jx, jy = mm(*JOY)
holes = [circle(jx, jy, JOY_OPEN_D)]
for name, i, j, shape in SWITCHES:
    x, y = mm(i, j)
    if shape == "square":
        s = CAP_SIZE / 2 + HOLE_CLEAR; r = CAP_CORNER + HOLE_CLEAR
        holes.append(rrect(x - s, y - s, x + s, y + s, r))
    else:
        holes.append(tri(x, y, shape == "tri_up", HOLE_CLEAR))
for (i, j) in MOUNT:
    holes.append(circle(*mm(i, j), LID_SCREW_D))
extrude(outer_box, holes, Z_WALL, Z_LID_TOP)                       # lid plate
for (i, j) in MOUNT:                                               # clamp posts / screw bosses
    x, y = mm(i, j)
    extrude(circle(x, y, LID_POST_D), [circle(x, y, LID_SCREW_D)], Z_PCB_TOP + POST_GAP, Z_WALL + 0.5)
lo = (ix0 + FIT, iy0 + FIT, ix1 - FIT, iy1 - FIT)
lip_o = rrect(*lo, max(CORNER_R - WALL - FIT, 0.5))
lip_i = rrect(lo[0] + LIP_T, lo[1] + LIP_T, lo[2] - LIP_T, lo[3] - LIP_T, 0.5)
extrude(lip_o, [lip_i], Z_WALL - LIP_H, Z_WALL + 0.5)              # locating lip
cy = PROMICRO["cy"]; h = USB_SLOT_W / 2 - FIT
extrude(np.array([[ox0, cy - h], [ix0 + 0.5, cy - h], [ix0 + 0.5, cy + h], [ox0, cy + h]]),
        [], TAB_BOT, Z_WALL + 0.5)                                 # tab that closes the slot
flip = [(np.array([-a[0], a[1], Z_LID_TOP - a[2]]),
         np.array([-b[0], b[1], Z_LID_TOP - b[2]]),
         np.array([-c[0], c[1], Z_LID_TOP - c[2]])) for a, b, c in TRIS]
write_stl("lid.stl", flip)

print(f"board {bx1-bx0:.1f} x {by1-by0:.1f} mm ({round((bx1-bx0)/G)} x {round((by1-by0)/G)} holes)")
print(f"box outer {ox1-ox0:.1f} x {oy1-oy0:.1f} x {Z_WALL:.1f} mm, lid top at {Z_LID_TOP:.1f}")
print("box tris", len(box_tris), "lid tris", len(flip))
print(f"lid screws: M3 x {LID_T + (Z_WALL - Z_PCB_TOP) + PCB_T:.1f} mm to reach the standoff top -> use M3x25 (≈{25 - (LID_T + (Z_WALL - Z_PCB_TOP) + PCB_T):.1f} mm into standoff)")
# clearance: lid posts vs keycap flanges (13.5 mm rounded squares) and switch bodies
FL, FLR = 13.5, CAP_CORNER * 13.5 / CAP_SIZE
for (i, j) in MOUNT:
    px, py = mm(i, j)
    for name, si, sj, shape in SWITCHES:
        sx, sy = mm(si, sj)
        for half, rr, what in ((FL / 2, FLR, "cap flange"), (6.0, 0.3, "switch body")):
            dx = max(abs(px - sx) - (half - rr), 0); dy = max(abs(py - sy) - (half - rr), 0)
            d = math.hypot(dx, dy) - rr - LID_POST_D / 2
            if d < 1.0: print(f"  post ({i},{j}) to {name} {what}: {d:.2f} mm clearance")

# ================= layout drawing =================
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
fig, ax = plt.subplots(figsize=(13, 7.5))
ax.add_patch(Polygon(outer_box, fc="#e9e7e1", ec="#999", lw=0.8))
ax.add_patch(Polygon(inner_box, fc="#f7f6f2", ec="#bbb", lw=0.6))
ax.add_patch(Rectangle((bx0, by0), bx1 - bx0, by1 - by0, fc="#c98b45", ec="#7a4b1a", lw=1, alpha=0.55))
ii = np.arange(-1, round(bx1 / G) + 1); jj = np.arange(-1, round(by1 / G) + 1)
gx, gy = np.meshgrid(ii * G, jj * G)
ax.plot(gx.ravel(), gy.ravel(), ".", color="#6b3d12", ms=1.6)
pm = PROMICRO
ax.add_patch(Rectangle((pm["x0"], pm["cy"] - pm["w"] / 2), pm["len"], pm["w"], fc="none", ec="#2a6fd6", ls="--", lw=1.2))
ax.text(pm["x0"] + pm["len"] / 2, pm["cy"] - pm["w"] / 2 + 1.2, "Pro Micro (underneath)\nUSB → left", color="#2a6fd6", ha="center", fontsize=8)
x, y = mm(*JOY)
JM = joy_mount_holes()
mx0 = min(p[0] for p in JM) - 3.5; mx1 = max(p[0] for p in JM) + 3.5
my0 = min(p[1] for p in JM) - 4.5; my1 = max(p[1] for p in JM) + 4.5
ax.add_patch(Rectangle((mx0, my0), mx1 - mx0, my1 - my0, fc="#222", ec="#000", alpha=0.15, ls="--"))
ax.text((mx0 + mx1) / 2, my0 + 1, "module (approx. outline)\nheader → front", ha="center", fontsize=6.5, color="#333")
ax.add_patch(Circle((x, y), JOY_DOME_D / 2, fc="none", ec="#555", ls=":", lw=1))
ax.add_patch(Circle((x, y), JOY_OPEN_D / 2, fc="none", ec="#888", ls="-.", lw=0.8))
ax.plot(x, y, "+", color="#c00", ms=14, mew=2)
ax.text(x, y + 3, f"shaft ({JOY[0]},{JOY[1]})", ha="center", fontsize=7.5, color="#c00")
for (hx, hy) in JM:
    ax.add_patch(Circle((hx, hy), JOY_MOUNT_D / 2, fc="w", ec="#7b1fa2", lw=1.5))
    ax.text(hx, hy - 3.2 if hy < y else hy + 2.2, f"({hx - x:+.0f}, {hy - y:+.0f})", ha="center", fontsize=6.5, color="#7b1fa2")
for name, i, j, shape in SWITCHES:
    x, y = mm(i, j)
    ax.add_patch(Rectangle((x - 6, y - 6), 12, 12, fc="#333", ec="#000", alpha=0.75))
    for dx in (-2.5, 2.5):
        for dy in (-1, 1):
            ax.add_patch(Circle((x + dx * G, y + dy * G), 0.7, fc="#ffd400", ec="#000", lw=0.5))
    ax.text(x, y, name, ha="center", va="center", color="w", fontsize=11, weight="bold")
    ax.text(x, y - 8.3, f"legs i {i-2.5:g},{i+2.5:g}  j {j-1:g},{j+1:g}", ha="center", fontsize=7)
for (i, j) in MOUNT + MID_STANDOFFS:
    x, y = mm(i, j)
    ax.add_patch(Circle((x, y), 1.5, fc="w", ec="#c00", lw=1.5))
    ax.text(x + 2.2, y + 2.2, f"M3 ({i},{j})", fontsize=7, color="#c00")
cy = PROMICRO["cy"]
ax.add_patch(Rectangle((ox0 - 0.2, cy - USB_SLOT_W / 2), WALL + 0.4, USB_SLOT_W, fc="#2a6fd6", alpha=0.5))
ax.text(ox0 - 1, cy, "USB\nslot", ha="right", va="center", fontsize=8, color="#2a6fd6")
ax.set_title(f"Board {round((bx1-bx0)/G)} × {round((by1-by0)/G)} holes ({bx1-bx0:.1f} × {by1-by0:.1f} mm), "
             "top view, front edge at bottom. Coordinates = holes from front-left mounting hole.", fontsize=10)
ax.set_aspect("equal"); ax.set_xlim(ox0 - 14, ox1 + 3); ax.set_ylim(oy0 - 3, oy1 + 3); ax.axis("off")
ax.text(ox0 - 13, oy0 - 1.5, "purple = joystick module holes, mm from shaft centre.  dotted circle = 26 mm dome.  dash-dot = lid opening.",
        fontsize=7.5, color="#7b1fa2")
plt.savefig("board_layout.png", dpi=130, bbox_inches="tight")

# ---------- 1:1 drilling template (A4 landscape, print at 100% / actual size) ----------
from matplotlib.backends.backend_pdf import PdfPages
W, H = 297 / 25.4, 210 / 25.4
fig = plt.figure(figsize=(W, H)); ax = fig.add_axes([0, 0, 1, 1])
cx, cy0 = 148.5 - (bx0 + bx1) / 2, 105 - (by0 + by1) / 2
T = lambda px, py: (px + cx, py + cy0)
ax.add_patch(Rectangle(T(bx0, by0), bx1 - bx0, by1 - by0, fc="none", ec="k", lw=0.8))
for i in ii:
    for j in jj:
        if bx0 - 0.1 <= i * G <= bx1 + 0.1 and by0 - 0.1 <= j * G <= by1 + 0.1:
            ax.add_patch(Circle(T(i * G, j * G), 0.45, fc="#bbb", ec="none"))
def drill(px, py, d, col):
    X, Y = T(px, py)
    ax.add_patch(Circle((X, Y), d / 2, fc="none", ec=col, lw=0.8))
    ax.plot([X - d, X + d], [Y, Y], color=col, lw=0.4); ax.plot([X, X], [Y - d, Y + d], color=col, lw=0.4)
for (i, j) in MOUNT + MID_STANDOFFS: drill(i * G, j * G, 3.0, "#c00")
for (hx, hy) in JM: drill(hx, hy, JOY_MOUNT_D, "#7b1fa2")
x, y = mm(*JOY)
ax.add_patch(Circle(T(x, y), JOY_DOME_D / 2, fc="none", ec="#999", ls=":", lw=0.6))
ax.plot(*T(x, y), "+", color="#c00", ms=10)
ax.add_patch(Rectangle(T(mx0, my0), mx1 - mx0, my1 - my0, fc="none", ec="#777", ls="--", lw=0.5))
for name, i, j, shape in SWITCHES:
    X, Y = T(*mm(i, j))
    ax.add_patch(Rectangle((X - 6, Y - 6), 12, 12, fc="none", ec="#333", lw=0.6))
    ax.text(X, Y, name, ha="center", va="center", fontsize=9)
ax.add_patch(Rectangle((20, 20), 50, 10, fc="none", ec="k", lw=0.8))
ax.text(45, 25, "50 mm - check scale", ha="center", va="center", fontsize=8)
ax.text(20, 190, "PTZ controller - 1:1 drilling template. Print at 100% / actual size. Red = board M3, purple = joystick M3. Front edge at bottom.",
        fontsize=8)
ax.set_xlim(0, 297); ax.set_ylim(0, 210); ax.axis("off")
fig.savefig("drill_template.pdf")
