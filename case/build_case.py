#!/usr/bin/env python3
"""Foot pod case: base + snap-on lid with lace tunnels underneath.

    pip install manifold3d trimesh numpy
    python build_case.py            # writes footpod_base.stl, footpod_lid.stl

Layout (base coordinates, mm): X = pod length (USB-C end at +X, point it at the toe),
Y = width, Z = up. The 402030 battery lies on the floor, the XIAO sits on top of it
on foam tape with its USB-C against the +X wall, and the JST plug pair sits on the
battery beside the board at the -X end.
"""
import numpy as np
import trimesh
from manifold3d import CrossSection, Manifold

# ---------------- parameters ----------------
WALL = 1.6
CAV_L, CAV_W, CAV_H = 34.0, 22.4, 9.4       # inner cavity
FLOOR_SKIN, TUNNEL_H, TUNNEL_CEIL = 0.8, 3.0, 0.8
TUNNEL_W, TUNNEL_X = 10.0, 10.0             # lace tunnel width, centre offset from middle
FLOOR_TOP = FLOOR_SKIN + TUNNEL_H + TUNNEL_CEIL   # 4.6
BASE_H = FLOOR_TOP + CAV_H                         # 14.0
OUT_L, OUT_W = CAV_L + 2 * WALL, CAV_W + 2 * WALL  # 37.2 x 25.6
R_OUT = 3.0
LID_T, LIP_H, LIP_T, LIP_GAP = 1.4, 2.0, 1.2, 0.15
USB_W, USB_H = 9.4, 3.8
# stack heights above the floor
BATT = (30.5, 20.0, 4.3)
FOAM = 0.4
BOARD = (21.0, 17.8, 1.0)
USB_Z = FOAM + BATT[2] + BOARD[2] + 1.6            # USB-C centre above floor top
SEG = 48


def rrect(l, w, r):
    r = min(r, l / 2 - 0.01, w / 2 - 0.01)
    c = CrossSection.circle(r, SEG)
    pts = [(sx * (l / 2 - r), sy * (w / 2 - r)) for sx in (-1, 1) for sy in (-1, 1)]
    return CrossSection.batch_hull([c.translate(p) for p in pts])


def box(cx, cy, cz, sx, sy, sz):
    return Manifold.cube([sx, sy, sz], center=True).translate([cx, cy, cz])


def base():
    shell = rrect(OUT_L, OUT_W, R_OUT).extrude(BASE_H)
    cavity = rrect(CAV_L, CAV_W, R_OUT - WALL).extrude(CAV_H + 1).translate([0, 0, FLOOR_TOP])
    m = shell - cavity
    # lace tunnels across the width (bridged when printed, no supports)
    for sx in (-1, 1):
        m -= box(sx * TUNNEL_X, 0, FLOOR_SKIN + TUNNEL_H / 2, TUNNEL_W, OUT_W + 2, TUNNEL_H)
        # soften the tunnel mouths so they don't saw the laces
        for sy in (-1, 1):
            m -= box(sx * TUNNEL_X, sy * OUT_W / 2, FLOOR_SKIN + TUNNEL_H / 2,
                     TUNNEL_W + 1.2, 1.2, TUNNEL_H + 1.2)
    # USB-C opening in the +X wall
    usb = rrect(USB_H, USB_W, 1.2).extrude(WALL * 3).rotate([0, 90, 0])
    m -= usb.translate([OUT_L / 2 - WALL * 2, 0, FLOOR_TOP + USB_Z])
    # detent grooves for the lid bumps (long walls)
    for sy in (-1, 1):
        m -= box(0, sy * (CAV_W / 2 + 0.25), BASE_H - 1.0, 14, 0.5 + 0.02, 1.2)
    return m


def lid():
    """Printed plate-down; lip points up. Flip it over to fit."""
    plate = rrect(OUT_L, OUT_W, R_OUT).extrude(LID_T)
    lo = rrect(CAV_L - 2 * LIP_GAP, CAV_W - 2 * LIP_GAP, R_OUT - WALL - LIP_GAP)
    li = rrect(CAV_L - 2 * (LIP_GAP + LIP_T), CAV_W - 2 * (LIP_GAP + LIP_T),
               max(0.3, R_OUT - WALL - LIP_GAP - LIP_T))
    lip = (lo.extrude(LIP_H) - li.extrude(LIP_H + 1).translate([0, 0, -0.5])).translate([0, 0, LID_T])
    # no lip on the USB end: it would hit the USB-C receptacle
    lip -= box(CAV_L / 2 - 2.0, 0, LID_T + LIP_H / 2, 6.0, CAV_W, LIP_H + 1)
    m = plate + lip
    # snap bumps on the long sides
    for sy in (-1, 1):
        y = sy * (CAV_W / 2 - LIP_GAP + 0.175)
        m += box(0, y, LID_T + 1.0, 12, 0.35, 0.8)
    return m


def to_trimesh(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts), process=True)


def components():
    """Rough envelopes of the parts, in base coordinates, for the fit check."""
    z0 = FLOOR_TOP
    inner_x = CAV_L / 2
    batt = box(0, 0, z0 + BATT[2] / 2, *BATT)
    bz = z0 + BATT[2] + FOAM
    board = box(inner_x - BOARD[0] / 2, 0, bz + BOARD[2] / 2, *BOARD)
    shield = box(inner_x - 12, 0, bz + BOARD[2] + 1.1, 12, 12, 2.2)
    usbc = box(inner_x - 3.7, 0, bz + BOARD[2] + 1.6, 7.4, 8.94, 3.2)
    jst = box(-inner_x + 6, 0, z0 + BATT[2] + 2.3, 9.0, 6.0, 4.6)
    return {"battery": batt, "board": board, "shield": shield, "usb-c": usbc, "jst": jst}


def fit_check(b, lid_placed):
    ok = True
    for name, c in components().items():
        for part_name, part in (("base", b), ("lid", lid_placed)):
            v = (c ^ part).volume()
            if v > 1e-3:
                ok = False
                print(f"  CLASH {name} x {part_name}: {v:.2f} mm^3")
    # lid against base (bumps are meant to sit in the grooves)
    v = (b ^ lid_placed).volume()
    print(f"  lid/base overlap {v:.2f} mm^3 (expected ~0: bumps sit in grooves)")
    print("  fit check:", "OK" if ok else "FAILED")
    return ok


if __name__ == "__main__":
    import sys
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    b, l = base(), lid()
    lid_placed = l.mirror([0, 0, 1]).translate([0, 0, BASE_H + LID_T])
    print(f"Outer size {OUT_L:.1f} x {OUT_W:.1f} x {BASE_H + LID_T:.1f} mm")
    fit_check(b, lid_placed)
    for name, m in (("footpod_base.stl", b), ("footpod_lid.stl", l)):
        t = to_trimesh(m)
        t.export(f"{out}/{name}")
        print(f"{name}: watertight={t.is_watertight}, volume={t.volume:.0f} mm^3, "
              f"bounds={np.round(t.extents, 1).tolist()}")
