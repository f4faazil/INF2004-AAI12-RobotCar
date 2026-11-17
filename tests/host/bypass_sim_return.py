"""Geometry check of the free-roam bypass: does the 200 x 250 mm car clip the box?

Frame: x right, y forward, origin at the axle midpoint when the car stops.
Heading phi is counter-clockwise (left positive): the car points along (-sin phi, cos phi).
The car body, relative to the axle: x -100..+100, y -90 (rear) .. +160 (front).
"""
import math
import itertools


def body(x, y, phi):
    c, s = math.cos(phi), math.sin(phi)
    pts = [(-100, -90), (100, -90), (100, 160), (-100, 160)]
    # right vector (c, s), forward vector (-s, c)
    return [(x + bx * c - by * s, y + bx * s + by * c) for bx, by in pts]


def seg_dist(p, q, r, s):
    def inter(a, b, c, d):
        def o(p1, p2, p3):
            return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])
        return (o(a, b, c) * o(a, b, d) < 0) and (o(c, d, a) * o(c, d, b) < 0)
    if inter(p, q, r, s):
        return 0.0

    def pt_seg(a, b, c):
        dx, dy = c[0] - b[0], c[1] - b[1]
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((a[0] - b[0]) * dx + (a[1] - b[1]) * dy) / L))
        return math.hypot(a[0] - (b[0] + t * dx), a[1] - (b[1] + t * dy))
    return min(pt_seg(p, r, s), pt_seg(q, r, s), pt_seg(r, p, q), pt_seg(s, p, q))


def poly_dist(A, B):
    d = 1e9
    for i in range(len(A)):
        for j in range(len(B)):
            d = min(d, seg_dist(A[i], A[(i + 1) % len(A)], B[j], B[(j + 1) % len(B)]))
    return d


def inside(pt, poly):
    n = len(poly)
    c = False
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > pt[1]) != (y2 > pt[1]) and pt[0] < (x2 - x1) * (pt[1] - y1) / (y2 - y1) + x1:
            c = not c
    return c


def min_clearance(steps, box, turn_err_deg):
    """steps: list of ('T', deg) / ('M', mm).  Returns the smallest gap (mm), negative = overlap."""
    x = y = 0.0
    phi = 0.0
    best = 1e9
    errs = iter(turn_err_deg)
    for kind, v in steps:
        if kind == "T":
            target = phi + math.radians(v + next(errs))
            n = max(1, int(abs(target - phi) / math.radians(2)))
            for i in range(1, n + 1):
                p = phi + (target - phi) * i / n
                B = body(x, y, p)
                d = poly_dist(B, box)
                if any(inside(c, box) for c in B) or any(inside(c, B) for c in box):
                    d = -1.0
                best = min(best, d)
            phi = target
        else:
            hx, hy = -math.sin(phi), math.cos(phi)
            n = max(1, int(abs(v) / 5))
            for i in range(1, n + 1):
                px, py = x + hx * v * i / n, y + hy * v * i / n
                B = body(px, py, phi)
                d = poly_dist(B, box)
                if any(inside(c, box) for c in B) or any(inside(c, B) for c in box):
                    d = -1.0
                best = min(best, d)
            x, y = x + hx * v, y + hy * v
    return best, (x, y, math.degrees(phi))


def manoeuvre(side, lateral, forward, after=600):
    s = 1 if side == "left" else -1
    return [("T", s * 90), ("M", lateral), ("T", -s * 90), ("M", forward),
            ("T", -s * 90), ("M", lateral), ("T", s * 90), ("M", after)]


cases = [
    ("as run: sideways 451, forward 903, box 150 wide", "right", 451, 903, 150, 100, 471),
    ("as run: sideways 451, forward 903, box 300 wide", "right", 451, 903, 300, 100, 471),
    ("proposed: forward +150 = 1053, box 150 wide", "right", 451, 1053, 150, 100, 471),
    ("proposed: forward +150 = 1053, box 300 wide", "right", 451, 1053, 300, 100, 471),
    ("as run, but a DEEP box (250 mm deep, 150 wide)", "right", 451, 903, 150, 250, 471),
    ("proposed, DEEP box (250 mm deep, 150 wide)", "right", 451, 1053, 150, 250, 471),
]
SENSOR_AHEAD = 80      # the sensor face is 80 mm ahead of the axle
for name, side, lat, fwd, w, depth, closest in cases:
    y0 = closest + SENSOR_AHEAD
    box = [(-w / 2, y0), (w / 2, y0), (w / 2, y0 + depth), (-w / 2, y0 + depth)]
    print(name, "- planner:", side, "sideways", lat, "forward", fwd, "| box front %d mm ahead of the axle" % y0)
    for label, errs in (("perfect turns", (0, 0, 0, 0)), ("every turn +2.5 deg", (2.5, 2.5, 2.5, 2.5)),
                        ("every turn -2.5 deg", (-2.5, -2.5, -2.5, -2.5)), ("every turn -4 deg", (-4, -4, -4, -4)), ("+8 / -8 / +8 / -8", (8, -8, 8, -8))):
        gap, end = min_clearance(manoeuvre(side, lat, fwd), box, errs)
        print("   %-22s smallest gap to the box %5.0f mm   end pose x=%4.0f y=%5.0f heading %4.0f deg" % (label, gap, end[0], end[1], end[2]))
