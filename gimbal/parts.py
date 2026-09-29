"""Part builders for the Perigee gimbal (v8: goBILDA Stingray direct drives, round columns and shoulder drums, bolts + captive nuts).

World coordinates at the nominal pose (az = 0, el = 0, boresight +Y, az axis Z, el axis X).
Drives: a Stingray-4 (450 deg) under the deck turns the head directly, a Stingray-9 (200 deg) in the right shoulder turns the
cradle directly. Both are modelled as measured envelopes of goBILDA's STEP files (vendor/), with their output standoffs,
gears, pinions and the M4 holes that receive screws.
Fastening rule: no threads in plastic. Every bolt ends in a hex nut in a blind hex pocket or a side-loaded nut trap, or in the
gearboxes' own M4 threads.
"""
import math
import FreeCAD as App
import Part
from . import geom as G
from .geom import V, Z, X, Y
from .hardware import Hardware, NUT

COLORS = {
    "black": (0.09, 0.09, 0.10, 0.0), "orange": (1.00, 0.45, 0.05, 0.0), "white": (0.93, 0.93, 0.92, 0.0),
    "grey": (0.42, 0.44, 0.47, 0.0), "clear": (0.80, 0.90, 1.00, 0.55), "ref": (0.55, 0.60, 0.65, 0.75),
    "steel": (0.75, 0.76, 0.78, 0.0), "brass": (0.80, 0.62, 0.25, 0.0),
}
UP, DOWN, PX, NX, PY, NY = (0, 0, 1), (0, 0, -1), (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)

# goBILDA Stingray servo gearbox, measured from vendor/3215-0001-0004 and -0009 (identical apart from the gear pair).
# Normalised frame: output axis = +Z, load face (standoff ends, shaft end) at z = 0, gearbox below it, servo toward -X.
STINGRAY = {
    "block": (-69.8, 21.8, -21.8, 21.8, -32.6, -12.2),     # aluminium block x0 x1 y0 y1 z0 z1 (91.6 x 43.6 x 20.3)
    "servo": (-65.3, -11.0, -10.3, 10.3, -50.9, -11.0),     # 2000-series servo body standing through the block
    "gear_z": (-10.2, -4.2),                                 # output gear faces
    "hub_r": 16.0, "hub_z": -2.45,                           # hub boss between gear and load face
    "shaft_r": 5.5,                                          # hub-shaft end (8 mm REX) flush with the load face
    "standoff": 8.0, "standoff_af": 7.0,                     # 4x M4 standoffs on a 16 mm square, load interface
    "pinion_x": -48.0, "pinion_z": (-10.4, -4.4),
    "hole_rows_x": (-64.0, -32.0, -16.0, 16.0), "hole_y": 16.0,   # M4 through holes along Z (goBILDA 16 mm grid)
    "side_tap_x": (-64.0, -32.0), "side_tap_z": (-20.2, -28.2),   # M4 tapped holes on the +-Y faces
    "clear": 0.5,                                            # pocket clearance around the envelope
}
# gear_r = output gear tip radius; pinion_r is drawn at root radius (centre distance 48) so the two placeholder parts only touch
STINGRAY_KIND = {"4": dict(gear_r=39.2, pinion_r=8.6, label="Stingray-4 (4:1, 450 deg, 100 kg.cm)", sku="3215-0001-0004"),
                 "9": dict(gear_r=44.0, pinion_r=3.8, label="Stingray-9 (9:1, 200 deg, 227 kg.cm)", sku="3215-0001-0009")}


def fillet_edges(shape, radius, pick, label=""):
    """Fillet the edges selected by `pick`; on any failure return the input unchanged and say so in the build log."""
    try:
        edges = [e for e in shape.Edges if pick(e)]
        if not edges:
            if label:
                print("fillet %s: no edges matched" % label)
            return shape
        f = shape.makeFillet(radius, edges)
        if f.isValid() and len(f.Solids) == 1:
            return f
        if label:
            print("fillet %s: result invalid, skipped" % label)
    except Exception as ex:
        if label:
            print("fillet %s: failed (%s), skipped" % (label, str(ex).strip()[:60]))
    return shape


def side_name(sgn):
    return "R" if sgn > 0 else "L"


class Layout:
    def __init__(self, P):
        self.P = P
        self.z_floor = P["base_floor"]
        self.z_puck0 = P["puck_z0"]; self.z_deck = self.z_puck0 + P["puck_h"]
        self.R_lip = P["puck_od"] / 2; self.R_body = P["puck_body_od"] / 2; self.z_lip1 = self.z_puck0 + P["puck_lip_h"]
        self.R_base = P["base_od"] / 2; self.R_pedestal = P["base_body_od"] / 2; self.R_bore = P["base_wall_inner_r"]; self.R_thrust = P["thrust_ring_r"]
        self.z_base_top = P["base_h"]; self.z_ret1 = self.z_base_top + P["retainer_h"]
        self.R_drum = P["drum_od"] / 2; self.z_drum1 = self.z_deck + P["drum_h"]
        self.z_plate1 = self.z_drum1 + P["yoke_plate_h"]
        self.z_el = self.z_deck + P["el_axis_height"]
        self.x_arm_in = P["arm_inner_x"]; self.x_arm_out = self.x_arm_in + P["arm_thick"]; self.x_arm_mid = (self.x_arm_in + self.x_arm_out) / 2
        self.arm_r = P["arm_thick"] / 2                          # arms are round columns
        self.r_ret_bolt = (self.R_bore + self.R_pedestal) / 2   # retainer bolts run down the middle of the pedestal wall
        self.r_mount_bolt = (self.R_pedestal + self.R_base) / 2  # M6 mount holes in the foot flange
        self.r_axle = P["el_axle_d"] / 2; self.r_bush = P["el_bushing_od"] / 2
        self.R_sh = P["shoulder_r"]; self.x_sh_out = self.x_arm_in + P["shoulder_thick"]
        self.sh_drop = P["shoulder_drop"]; self.sh_wall = P["shoulder_wall"]; self.arm_wall = P["arm_wall"]   # stadium drop below the axis, shell and column walls
        self.arm_splits = [self.z_plate1, self.z_el - (P["segment_max"] - 30), self.z_el]   # upper piece + 24 mm spigot stays under segment_max
        self.hub = P["hub_size"]; self.hub_wall = P["hub_wall"]
        self.x_stub_end = self.x_sh_out - 12                    # left stub axle end (magnet)
        self.y_boom_end = P["dish_offset"]; self.y_cw_end = -(self.hub / 2 + P["cw_arm_len"])
        self.pillar_r = self.R_body - 12.5                     # pillars stand on the puck body, tied to the drum wall by ribs
        self.r_adapter_bolt = 25.5   # boom end flange bolts, on the axes so the M5 nut pockets sit outside the 40 mm square tube
        S = STINGRAY
        # EL Stingray-9: load face on the hub's right wall, gearbox in the right shoulder, servo below the axis
        self.x_el_load = self.hub / 2
        self.z_el_servo = self.z_el + (S["servo"][0] + S["servo"][1]) / 2       # servo centre height (cover axis)
        self.x_el_block = (self.x_el_load - S["block"][5], self.x_el_load - S["block"][4])     # 57.2 .. 77.6
        self.x_el_servo1 = self.x_el_load - S["servo"][4]                        # outer end of the servo, 95.9
        # AZ Stingray-4: load face on the puck underside, servo toward +Y
        self.z_az_block0 = self.z_puck0 + S["block"][4]                          # block underside, pillars stand under it



def _solidify(shape):
    """Boolean chains return one-solid Compounds; hand the registry the bare closed Solid so STEP/STL/FCStd carry a
    real solid. Multi-solid references (feed placeholder) are left as compounds."""
    if shape.ShapeType != "Solid" and len(shape.Solids) == 1:
        s = shape.Solids[0]
        if s.isValid():
            return s
    return shape


def _reg(reg, name, shape, group, color, qty=1, print_up=(0, 0, 1), notes="", printed=True, hidden=False):
    shape = _solidify(shape)
    reg[name] = dict(shape=shape, group=group, color=color, qty=qty, print_up=print_up, notes=notes, printed=printed, hidden=hidden)
    return shape


# ============================================================== STINGRAY placeholders
def stingray_solid(kind, part):
    """Envelope of a Stingray gearbox in the normalised frame (see STINGRAY), with the holes that receive screws.
    part = "static": block, servo, pinion (stays with the mount); "output": gear, hub, standoffs, shaft (turns with the load)."""
    S, K = STINGRAY, STINGRAY_KIND[kind]
    b, sv = S["block"], S["servo"]
    gz0, gz1 = S["gear_z"]
    if part == "static":
        block = G.box(b[1] - b[0], b[3] - b[2], b[5] - b[4], (b[0], b[2], b[4]))
        block = fillet_edges(block, 2.0, lambda e: abs(e.Vertexes[0].Z - e.Vertexes[1].Z) > 10, "stingray block")
        servo = G.box(sv[1] - sv[0], sv[3] - sv[2], sv[5] - sv[4], (sv[0], sv[2], sv[4]))
        pinion = G.cyl(K["pinion_r"], S["pinion_z"][1] - S["pinion_z"][0], (S["pinion_x"], 0, S["pinion_z"][0]))
        servo_shaft = G.cyl(3.0, S["pinion_z"][1] - sv[5], (S["pinion_x"], 0, sv[5]))
        s = G.fuse(block, servo, pinion, servo_shaft)
        s = s.cut(G.cyl(S["shaft_r"] + 0.3, b[5] - b[4] + 2, (0, 0, b[4] - 1)))     # bearing bore for the hub-shaft
        for rx in S["hole_rows_x"]:                                                # M4 through holes along Z (tapped; screws pass through)
            for sy in (-S["hole_y"], S["hole_y"]):
                s = s.cut(G.cyl(2.0, b[5] - b[4] + 2, (rx, sy, b[4] - 1)))
        for tx in S["side_tap_x"]:                                                 # M4 tapped holes on the +-Y faces, 10 deep
            for tz in S["side_tap_z"]:
                for sy in (-1, 1):
                    s = s.cut(G.cyl(2.0, 10, (tx, sy * b[3], tz), (0, -sy, 0)))
        return s
    gear = G.cyl(K["gear_r"], gz1 - gz0, (0, 0, gz0))
    for r_ in (24, 32):                                       # goBILDA lightening pattern, cosmetic
        for p in G.pattern_circle(8 if r_ < 30 else 12, r_, gz0 - 1, 0):
            gear = gear.cut(G.cyl(2.0, gz1 - gz0 + 2, p))
    hub = G.cyl(S["hub_r"], S["hub_z"] - gz1, (0, 0, gz1))                     # r16 boss from the gear face to the load face region
    shaft = G.cyl(S["shaft_r"], -S["hub_z"], (0, 0, S["hub_z"]))
    hub_shaft = G.cyl(S["shaft_r"], -b[4], (0, 0, b[4]))                       # 8 mm REX hub-shaft through the block bearings
    s = G.fuse(gear, hub, shaft, hub_shaft)
    a = S["standoff"]
    for sx, sy in ((-a, -a), (a, -a), (-a, a), (a, a)):
        s = G.fuse(s, G.hex_prism(S["standoff_af"], -gz1, (sx, sy, gz1)))
        s = s.cut(G.cyl(2.0, 8, (sx, sy, -8)))                                # M4 thread in the standoff, 8 deep from the load face
    return s


def placed_stingray(kind, at, x_dir, z_dir, part):
    """Place a normalised Stingray: its +Z (load direction) along z_dir, its +X (servo is at -X) along x_dir."""
    zd = V(*z_dir); zd.normalize(); xd = V(*x_dir); xd.normalize(); yd = zd.cross(xd)
    m = App.Matrix()
    m.A11, m.A21, m.A31 = xd.x, xd.y, xd.z
    m.A12, m.A22, m.A32 = yd.x, yd.y, yd.z
    m.A13, m.A23, m.A33 = zd.x, zd.y, zd.z
    m.A14, m.A24, m.A34 = at
    s = stingray_solid(kind, part)
    s.transformShape(m)
    return s, m


def stingray_pocket(kind, at, x_dir, z_dir, extra_servo=None):
    """Pocket cutter for a placed Stingray: block and servo envelopes grown by the clearance, plus the gear disc."""
    S, K = STINGRAY, STINGRAY_KIND[kind]; c = S["clear"]
    b, sv = S["block"], S["servo"]
    cutter = G.fuse(G.box(b[1] - b[0] + 2 * c, b[3] - b[2] + 2 * c, b[5] - b[4] + 2 * c, (b[0] - c, b[2] - c, b[4] - c)),
                    G.box(sv[1] - sv[0] + 2 * c, sv[3] - sv[2] + 2 * c, sv[5] - sv[4] + 2 * c, (sv[0] - c, sv[2] - c, sv[4] - c)),
                    G.cyl(K["gear_r"] + 3, -S["gear_z"][0] + 2, (0, 0, S["gear_z"][0] - 1)))
    zd = V(*z_dir); zd.normalize(); xd = V(*x_dir); xd.normalize(); yd = zd.cross(xd)
    m = App.Matrix()
    m.A11, m.A21, m.A31 = xd.x, xd.y, xd.z
    m.A12, m.A22, m.A32 = yd.x, yd.y, yd.z
    m.A13, m.A23, m.A33 = zd.x, zd.y, zd.z
    m.A14, m.A24, m.A34 = at
    cutter.transformShape(m)
    return cutter


# ============================================================== BASE
def build_base(P, L, reg, hw):
    S = STINGRAY
    base = G.fuse(G.cyl(L.R_base, 10), G.cyl(L.R_pedestal, L.z_base_top))
    base = fillet_edges(base, 3.0, lambda e: e.Curve.TypeId == "Part::GeomCircle" and abs(e.Curve.Radius - L.R_base) < 1e-3 and abs(e.Vertexes[0].Z - 10) < 1e-3, "base flange")
    base = base.cut(G.cyl(L.R_thrust, L.z_base_top, (0, 0, L.z_floor)))
    base = base.cut(G.cyl(L.R_bore, L.z_base_top, (0, 0, L.z_puck0)))
    for ang in (30, 150, 270):
        base = base.cut(G.rot(G.cbox(6, 22, 14, (L.R_pedestal - 1.5, 0, 26)), Z, ang))
    base = base.cut(G.box(10, L.R_base + 5, 8, (-5, -(L.R_base + 5), L.z_floor)))          # cable slot to the outside, -Y
    # AZ Stingray-4 sits on four pillars, bolted from under the floor into its M4 grid holes (rows y = +64 / -16, x = +-16)
    az_at, az_x, az_z = (0, 0, L.z_puck0), (0, -1, 0), (0, 0, 1)                             # servo toward +Y
    for py_ in (64.0, -16.0):
        for sx in (-1, 1):
            hx = sx * S["hole_y"]
            base = G.fuse(base, G.cbox(12, 12, L.z_az_block0 - L.z_floor, (sx * 18, py_, (L.z_az_block0 + L.z_floor) / 2)))
            base = hw.cbore_hole(base, 4, (hx, py_, 4.5), UP, L.z_az_block0, cbore_depth=4.5)
            hw.screw("Base", 4, 40, (hx, py_, 4.5), UP)
    # retainer flange plate: 12x M4 down into nut traps opening on the pedestal's outer face. Since v10 the ring is split
    # and only 8 of the traps hold nuts (the hooks); the traps at 15 / 165 / 195 / 345 deg stay empty and their screws
    # and nuts join the two ring halves at the lap notches instead.
    ret_pts = G.pattern_circle(P["retainer_bolts"], L.r_ret_bolt, L.z_base_top, 15)
    hook_k = [k for k in range(P["retainer_bolts"]) if k % 6 not in (0, 5)]
    for k, p in enumerate(ret_pts):
        base = hw.clear_hole(base, 4, p, DOWN, 12)
        rd = (p[0] / L.r_ret_bolt, p[1] / L.r_ret_bolt, 0)
        base = hw.nut_trap(base, 4, (p[0], p[1], L.z_base_top - 9), DOWN, rd, 10)
        if k in hook_k:
            hw.nut("Base", 4, (p[0], p[1], L.z_base_top - 9 + NUT[4][1] / 2), DOWN, open_dir=rd)
    for p in G.pattern_circle(6, L.r_mount_bolt, 0, 30):
        base = base.cut(G.cyl(3.3, 12, p))
    _reg(reg, "Base_Cup", base, "Base", "black", notes="AZ pedestal. Step at the puck underside (r 84-96) is the greased thrust face; 192 mm bore is the journal. Stingray-4 on four pillars, 4x M4x40 from under the floor. 6x M6 in the foot flange for a mount plate.")

    # ---- retainer ring, split (v10) so it goes on last: the drum overhangs the bolt circle with only 6 mm above the
    # ring, so no key reaches a ring screw once the head is in. Half A (+Y) and half B (-Y) slide in sideways through
    # that gap, under 8 hook screws pre-set to height in the pedestal nuts (open slots run from each hook in to the
    # bore along the slide direction), and meet at two half-lap notches on the X axis, bolted outside the drum's reach.
    z0, z1 = L.z_base_top, L.z_ret1
    lap, fit = P["retainer_lap"], 0.2                  # notch half-length along Y, sliding clearance
    z_lap = z0 + P["retainer_lap_lower"]               # top of A's lower tongue; B's upper tongue starts `fit` above
    ret = G.tube(L.R_base, L.R_body + 0.5, P["retainer_h"], (0, 0, z0))
    ret = fillet_edges(ret, 2.5, lambda e: e.Curve.TypeId == "Part::GeomCircle" and abs(e.Curve.Radius - L.R_base) < 1e-3 and abs(e.Vertexes[0].Z - z1) < 1e-3, "retainer")
    big = 2 * L.R_base + 40
    slab = lambda y0, y1, za, zb: G.box(big, y1 - y0, zb - za, (-big / 2, y0, za))
    ra = ret.common(G.fuse(slab(lap + fit, big / 2, z0 - 1, z1 + 1), slab(-lap, lap + fit, z0 - 1, z_lap)))
    rb = ret.common(G.fuse(slab(-big / 2, -lap - fit, z0 - 1, z1 + 1), slab(-lap - fit, lap, z_lap + fit, z1 + 1)))
    r_in = L.R_body + 0.5                              # tongue tips cross the axis while sliding: trim them straight at
    ra = ra.cut(G.box(2 * r_in, lap + 2, P["retainer_h"] + 2, (-r_in, -lap - 2, z0 - 1)))   # |x| = r_in so they
    rb = rb.cut(G.box(2 * r_in, lap + 2, P["retainer_h"] + 2, (-r_in, 0, z0 - 1)))          # clear the puck body
    wc, wh = 4.5, 8.2                                  # shank slot = M4 clearance, head groove = the old counterbore
    for k, p in enumerate(ret_pts):                    # slots run from each hook toward the axis and open at the bore
        if k not in hook_k:
            continue
        sy = 1 if p[1] > 0 else -1
        run = abs(p[1])
        y0 = 0.0 if sy > 0 else p[1]
        slot = G.fuse(G.cyl(wc / 2, P["retainer_h"] + 2, (p[0], p[1], z0 - 1)), G.box(wc, run, P["retainer_h"] + 2, (p[0] - wc / 2, y0, z0 - 1)),
                      G.cyl(wh / 2, 4.5, (p[0], p[1], z1 - 3.5)), G.box(wh, run, 4.5, (p[0] - wh / 2, y0, z1 - 3.5)))
        if sy > 0:
            ra = ra.cut(slot)
        else:
            rb = rb.cut(slot)
        hw.screw("Base", 4, 20, (p[0], p[1], z1 - 3.5), DOWN)
    for sx in (-1, 1):                                 # notch bolts: down through B's tongue and A's tongue into a nut
        for jy in (-P["retainer_joint_dy"], P["retainer_joint_dy"]):   # pushed up into a hex pocket under A, outside the pedestal
            x = sx * P["retainer_joint_r"]
            rb = hw.clear_hole(rb, 4, (x, jy, z1), DOWN, P["retainer_h"])
            ra = hw.clear_hole(ra, 4, (x, jy, z_lap), DOWN, P["retainer_h"])
            ra = hw.nut_pocket(ra, 4, (x, jy, z0), UP)
            hw.screw("Base", 4, 20, (x, jy, z1), DOWN)
            hw.nut("Base", 4, (x, jy, z0 + NUT[4][1] + 0.25), DOWN)
    _reg(reg, "Az_Retainer_Ring_A", ra, "Base", "black", notes="Retainer half A (+Y), goes in first: slides in from +Y under the drum, 4 open slots pass the pre-set hook screws (M4x20 into the pedestal nuts); lower tongue of both lap notches with 4 hex pockets underneath for the notch nuts.")
    _reg(reg, "Az_Retainer_Ring_B", rb, "Base", "black", notes="Retainer half B (-Y), goes in second: slides in from -Y over A's tongues, 4 open slots pass the hook screws; notch bolts 4x M4x20 down through both tongues. Print with support under the two tongues.")

    az, _ = placed_stingray("4", az_at, az_x, az_z, "static")
    _reg(reg, "Az_Stingray4", az, "Base", "black", printed=False, notes="goBILDA %s, SKU %s: direct AZ drive under the deck (block + servo + pinion)." % (STINGRAY_KIND["4"]["label"], STINGRAY_KIND["4"]["sku"]))
    azo, _ = placed_stingray("4", az_at, az_x, az_z, "output")
    _reg(reg, "Az_Stingray4_Output", azo, "Head", "steel", printed=False, notes="Stingray-4 output gear, hub and standoffs: turns with the head, bolted to the puck (4x M4x25 from the deck).")


# ============================================================== ARMS (round columns, hollow stadium shoulder drums)
def build_arms(P, L, reg, hw):
    """Yoke columns and shoulder drums (v9).
    Columns: 60 mm round tubes, 3 mm walls, lower + upper segment joined by a spigot with 2x M4x50 across.
    Shoulders: a 120 mm drum (R_sh) whose lower half is dropped `shoulder_drop` below the axis (stadium profile) so the
    Stingray-9 block (70 mm below the axis) fits. Both halves are 4 mm shells, closed on the outer face and open toward
    the hub, printed lying on the outer face. The Stingray-9 is screwed along its axis (6x M4 through its own tapped
    16 mm grid) into a 6 mm back plate; the caps take 2x M4x50 each into nuts 36 mm down in the lower half."""
    S = STINGRAY
    x0, xs = L.x_arm_in, L.x_sh_out
    xc, ra, wall = L.x_arm_mid, L.arm_r, L.arm_wall
    ri = ra - wall
    z0, z1, z2 = L.arm_splits
    Rk, drop, sw = L.R_sh, L.sh_drop, L.sh_wall
    y_wall = math.sqrt(ra ** 2 - 16 ** 2)                        # tube surface offset at the cross-bolt row (y = +-16)
    c = S["clear"]

    # ---- lower segment: hollow column, solid foot boss (cable through), socket for the upper segment's spigot
    low = G.cyl(ra, z1 - z0, (xc, 0, z0)).cut(G.cyl(ri, z1 - z0 - 16 + 1, (xc, 0, z0 + 16)))
    low = low.cut(G.cyl(8, 20, (xc, 0, z0 - 1)))                                              # cables from the drum bay up the arm
    for fx in (xc - 18, xc + 18):                                                              # feet: 4x M4x25 up from under the plate
        for fy in (-18, 18):
            od = ((fx - xc) / 18 / math.sqrt(2), fy / 18 / math.sqrt(2), 0)
            low = hw.clear_hole(low, 4, (fx, fy, z0), UP, 10)
            low = hw.nut_trap(low, 4, (fx, fy, z0 + 9), UP, od, 8, extra_len=5)
    # 2x M4x50 across the socket and spigot. On a round tube a shallow counterbore leaves the head proud where the
    # surface curves away, so head and nut are sunk 7 mm deep: below the tube surface, into the solid spigot behind the wall.
    x_head, x_nut = xc + y_wall - 7.0, xc - y_wall + 4.0                                     # nut sits on the 9 mm pocket floor
    for yy in (-16, 16):
        low = hw.cbore_hole(low, 4, (x_head, yy, z1 - 12), NX, 60, cbore_depth=10)
        low = hw.nut_pocket(low, 4, (xc - y_wall - 2, yy, z1 - 12), PX, depth=9)

    # ---- shoulder drum profile: half-circle R above the axis, straight sides for `drop`, half-circle R below
    def stadium_x(R, xa, xb):
        zc1, zc2 = z2, z2 - drop
        p = lambda y, z: V(xa, y, z)
        e1 = Part.ArcOfCircle(p(R, zc1), p(0, zc1 + R), p(-R, zc1)).toShape()
        e2 = Part.LineSegment(p(-R, zc1), p(-R, zc2)).toShape()
        e3 = Part.ArcOfCircle(p(-R, zc2), p(0, zc2 - R), p(R, zc2)).toShape()
        e4 = Part.LineSegment(p(R, zc2), p(R, zc1)).toShape()
        return Part.Face(Part.Wire([e1, e2, e3, e4])).extrude(V(xb - xa, 0, 0))
    below = G.box(xs - x0 + 6, 2 * Rk + 6, Rk + drop + 6, (x0 - 3, -Rk - 3, z2 - Rk - drop - 6))     # z <= z2
    above = G.box(xs - x0 + 6, 2 * Rk + 6, Rk + 6, (x0 - 3, -Rk - 3, z2))                               # z >= z2
    on_outer_face = lambda e: all(abs(v.X - xs) < 1e-3 for v in e.Vertexes) and not all(abs(v.Z - z2) < 1e-3 for v in e.Vertexes)
    outer_lo = fillet_edges(stadium_x(Rk, x0, xs).common(below), 2.5, on_outer_face, "drum lower rim")   # small: this edge sits on the bed
    outer_hi = fillet_edges(G.cyl(Rk, xs - x0, (x0, 0, z2), X).common(above), 2.5, on_outer_face, "drum cap rim")
    inner_lo = stadium_x(Rk - 1, x0, xs - 1).common(below)                              # clip for internal features: 1 mm inside the skin, so no shared faces
    inner_hi = G.cyl(Rk - 1, xs - 1 - x0, (x0, 0, z2), X).common(above)
    cav_lo = stadium_x(Rk - sw, x0 - 1, xs - sw).common(below)                         # open at the inner face and the split plane
    cav_hi = G.cyl(Rk - sw, xs - sw - (x0 - 1), (x0 - 1, 0, z2), X).common(G.mv(above, dz=sw))   # cap keeps a 4 mm split-plane plate
    x_bp = L.x_el_block[1] + c + 6.0                                                    # Stingray-9 back plate outer face (84.1)
    y_blk = S["block"][3] + c                                                           # block half-width with clearance (22.3)
    y_ch = y_blk + 3.0                                                                  # channel walls 3 mm

    # ---- upper segment base (both sides): column + spigot + lower shell, cap-bolt ribs, cross bolts, cap bolts
    up = G.fuse(G.cyl(ra, z2 - z1, (xc, 0, z1)), G.cyl(ri - 0.2, 24, (xc, 0, z1 - 24)), outer_lo)
    up = up.cut(cav_lo)
    up = up.cut(G.cyl(ri, (z2 - 85) - (z1 + 10), (xc, 0, z1 + 10)))                     # hollow column (10 mm plug above the spigot) ...
    up = up.cut(G.cyl(ri - 2.0, 26, (xc, 0, z2 - 86)))                                   # ... opening into the shell (2 mm narrower: stays clear of the outer plate)
    up = up.cut(G.cyl(8, 40, (xc, 0, z1 - 25)))                                          # cable bore through spigot and plug
    y_cb = Rk - 10.5                                                                      # cap bolts at y = +-49.5, in ribs against the shell wall
    for yy in (-y_cb, y_cb):
        rib = G.box(13, Rk - 44 + 2, Rk + drop, (xc - 6.5, 44 if yy > 0 else -Rk - 2, z2 - Rk - drop))
        up = G.fuse(up, rib.common(inner_lo))
    for yy in (-16, 16):                                                                  # same head / nut recesses as the socket
        up = hw.cbore_hole(up, 4, (x_head, yy, z1 - 12), NX, 70, cbore_depth=10)
        up = hw.nut_pocket(up, 4, (xc - y_wall - 2, yy, z1 - 12), PX, depth=9)
    z_cn = z2 - 36                                                                        # cap-bolt nut centre (M4x50 from z2 + 9 ends at z2 - 41)
    for yy in (-y_cb, y_cb):                                                              # cap bolts: traps open on the drum side
        up = hw.clear_hole(up, 4, (xc, yy, z2), DOWN, z2 - z_cn)
        up = hw.nut_trap(up, 4, (xc, yy, z_cn), DOWN, (0, 1 if yy > 0 else -1, 0), 14, extra_len=7)

    cap = outer_hi.cut(cav_hi)
    for yy in (-y_cb, y_cb):
        rib = G.box(13, Rk - 44 + 2, Rk, (xc - 6.5, 44 if yy > 0 else -Rk - 2, z2))
        cap = G.fuse(cap, rib.common(inner_hi))
        cap = hw.cbore_hole(cap, 4, (xc, yy, z2 + 9), DOWN, 10, cbore_depth=Rk)

    # ---- left: pillow block around the split bushing, on-axis encoder cap seat + 4x M3 (nuts slid in from the bore)
    pil = G.fuse(G.cyl(26, xs - x0, (x0, 0, z2), X), G.cyl(46, 14, (xs - 14, 0, z2), X))
    def left_features(shape, half):
        shape = G.fuse(shape, pil.common(half))
        shape = shape.cut(G.cyl(L.r_bush + 0.2, xs - x0 + 2, (x0 - 1, 0, z2), X))
        shape = shape.cut(G.cyl(40.2, 3, (xs - 3, 0, z2), X))
        return shape
    left = left_features(up, inner_lo)
    for ang in (205, 245, 295, 335):
        py, pz = 33 * math.cos(math.radians(ang)), 33 * math.sin(math.radians(ang))
        left = hw.clear_hole(left, 3, (xs - 3, py, z2 + pz), NX, 7)
        left = hw.nut_trap(left, 3, (xs - 8, py, z2 + pz), NX, (0, -py / 33, -pz / 33), 33 - L.r_bush - 0.5, extra_len=4)
    left = left.mirror(V(0, 0, 0), V(1, 0, 0))
    cap_left = left_features(cap, inner_hi).mirror(V(0, 0, 0), V(1, 0, 0))

    # ---- right: Stingray-9 channel (3 mm walls, floor, 6 mm back plate) inside the shell, window + lens on the outer face
    el_at, el_x, el_z = (L.x_el_load, 0, z2), (0, 0, 1), (-1, 0, 0)
    pocket = stingray_pocket("9", el_at, el_x, el_z)
    r_win, r_cov, r_ls = 38.0, 44.0, 40.5                                                # window (hex key reaches the row 32 below the axis), lens, lens screws
    window = G.cyl(r_win, sw + 2, (xs - sw - 1, 0, z2), X)
    seat = G.cyl(r_cov + 0.2, 3, (xs - 3, 0, z2), X)
    ring = G.tube(r_cov + 6, r_win - 8, 8.01, (xs - sw - 8, 0, z2), X)                   # thick ring behind the lens for its nut traps
    chan_lo = G.box(x_bp - x0, 2 * y_ch, Rk + drop + 2, (x0, -y_ch, z2 - Rk - drop - 1))
    for sy in (-1, 1):                                                                    # walls continue to the outer plate
        chan_lo = G.fuse(chan_lo, G.box(xs - sw - x_bp + 0.01, 3.0, Rk + drop + 2, (x_bp - 0.01, sy * y_blk if sy > 0 else -y_ch, z2 - Rk - drop - 1)))
    chan_hi = G.box(x_bp - x0, 2 * y_ch, y_blk + 4.0, (x0, -y_ch, z2))                  # block top wall + walls in the cap

    def right_features(shape, half, chan):
        shape = G.fuse(shape, ring.common(half), chan.common(half))
        shape = shape.cut(pocket).cut(window).cut(seat)
        for ang in (45, 135, 225, 315):                                                   # lens: 4x M3, nuts slid in from the window
            py, pz = r_ls * math.cos(math.radians(ang)), r_ls * math.sin(math.radians(ang))
            shape = hw.clear_hole(shape, 3, (xs - 3, py, z2 + pz), NX, 7)
            shape = hw.nut_trap(shape, 3, (xs - 8, py, z2 + pz), NX, (0, -py / r_ls, -pz / r_ls), r_ls - r_win + 2, extra_len=4)
        return shape
    right = right_features(up, inner_lo, chan_lo)
    cap_right = right_features(cap, inner_hi, chan_hi)
    # block screws: along the axis through the block's own tapped 16 mm grid holes into the back plate, driven through the window
    block_rows = ((-32.0, 25), (-16.0, 20), (16.0, 20))                                  # (grid row along the axis, screw length)
    for tx, length in block_rows:
        for sy in (-1, 1):
            seat_pt = (x_bp, sy * S["hole_y"], z2 + tx)
            tgt = cap_right if tx > 0 else right
            tgt = hw.clear_hole(tgt, 4, seat_pt, NX, 6.0)                                # head bears on the plate face
            if tx > 0:
                cap_right = tgt
            else:
                right = tgt
            hw.screw("Head", 4, length, seat_pt, NX)
    for ang in (45, 135, 225, 315):
        py, pz = r_ls * math.cos(math.radians(ang)), r_ls * math.sin(math.radians(ang))
        hw.nut("Head", 3, (xs - 8 - NUT[3][1] / 2, py, z2 + pz), PX, open_dir=(0, -py / r_ls, -pz / r_ls))

    # ---- register both sides (left = mirrored right-hand build)
    for side, sgn in (("R", 1), ("L", -1)):
        lo = low if sgn > 0 else low.mirror(V(0, 0, 0), V(1, 0, 0))
        # PERIGEE stencil through the outer wall, centred between the solid foot boss and the cross-bolt row
        zmid = ((z0 + 16) + (z1 - 12 - 5)) / 2
        mp = None if sgn > 0 else ((0, 0, zmid), (0, 1, 0))
        clip = G.box(ra + 1, 2 * ra + 2, z1 - z0, ((xc - 0.5) if sgn > 0 else -(xc + ra) - 0.5, -ra - 1, z0))     # outer half of the tube
        lo = G.stencil_text_cut(lo, P["logo_text"], P["logo_font"], P["logo_size"], 2 * ra + 20, at=((x0 - 5) if sgn > 0 else -(xs + 5), 0, zmid), axis=X, rotate_deg=180, mirror_plane=mp, clip=clip)
        for fx in (xc - 18, xc + 18):
            for fy in (-18, 18):
                hw.nut("Head", 4, (sgn * fx, fy, z0 + 9 - NUT[4][1] / 2), UP, open_dir=(sgn * (fx - xc), fy, 0))
        for yy in (-16, 16):
            hw.screw("Head", 4, 50, (sgn * x_head, yy, z1 - 12), (-sgn, 0, 0)); hw.nut("Head", 4, (sgn * x_nut, yy, z1 - 12), (sgn, 0, 0))
        _reg(reg, "Arm_%s_1" % side, lo, "Head", "white", print_up=(-sgn, 0, 0), notes="Lower arm column (%s), hollow (3 mm wall), cables inside. Feet 4x M4x25 from under the yoke plate into side-loaded nuts; socket for the upper spigot, 2x M4x50 across." % side)
        u = right if sgn > 0 else left
        for yy in (-y_cb, y_cb):
            hw.nut("Head", 4, (sgn * xc, yy, z_cn - NUT[4][1] / 2), UP, open_dir=(0, 1 if yy > 0 else -1, 0))
        if sgn < 0:
            for ang in (205, 245, 295, 335):
                py, pz = 33 * math.cos(math.radians(ang)), 33 * math.sin(math.radians(ang))
                hw.nut("Head", 3, (-(xs - 8 - NUT[3][1] / 2), py, z2 + pz), NX, open_dir=(0, -py / 33, -pz / 33))
        _reg(reg, "Arm_%s_2" % side, u, "Head", "white", print_up=(-sgn, 0, 0),
             notes="Upper arm column + lower half of the shoulder drum (%s), 4 mm shell open toward the hub: %s" % (side, "Stingray-9 channel with a 6 mm back plate (4x M4x20 + 2x M4x25 along the axis into the block's own threads, driven through the window), floor and cable opening into the column." if sgn > 0 else "pillow block for the left stub bushing, on-axis encoder cap."))
        cp = cap_right if sgn > 0 else cap_left
        for yy in (-y_cb, y_cb):
            hw.screw("Head", 4, 50, (sgn * xc, yy, z2 + 9), DOWN)
        _reg(reg, "El_Bearing_Cap_%s" % side, cp, "Head", "white", print_up=(-sgn, 0, 0), notes="Upper half of the shoulder drum, 4 mm shell with a split-plane plate, 2x M4x50 into nuts trapped in the lower half%s." % ("; holds the upper bushing half" if sgn < 0 else "; closes the top of the Stingray-9 channel and carries its upper screw row"))
    for half, zsgn in (("Lower", -1), ("Upper", 1)):
        bx0 = -(x0 + 42)
        bsh = G.tube(L.r_bush, L.r_axle + 0.25, 40, (bx0, 0, z2, ), X)
        bsh = bsh.common(G.box(40, 80, 40, (bx0, -40, z2 if zsgn > 0 else z2 - 40)))
        _reg(reg, "El_Bushing_L_%s" % half, bsh, "Head", "white", print_up=(0, 0, zsgn), notes="Half of the split plain bushing (36/30.5 x 40). Grease.")

    # ---- left axis cap with the AS5600 column and the status window
    xso = -xs
    cap = G.cyl(40, 7, (xso - 4, 0, L.z_el), X)
    cap = fillet_edges(cap, 2.5, lambda e: e.Curve.TypeId == "Part::GeomCircle" and abs(e.Curve.Radius - 40) < 1e-3 and abs(e.Vertexes[0].X - (xso - 4)) < 1e-3, "axis cap")
    cap = cap.cut(G.tube(41, 38.5, 1.2, (xso - 4, 0, L.z_el), X))
    for ang in (205, 245, 295, 335):
        py, pz = 33 * math.cos(math.radians(ang)), 33 * math.sin(math.radians(ang))
        seat_pt = (xso - 4 + 3.0, py, L.z_el + pz)
        cap = hw.cbore_hole(cap, 3, seat_pt, PX, 3.0 + 4, cbore_depth=3.0); hw.screw("Head", 3, 12, seat_pt, PX)
    x_board = -(L.x_stub_end) - 2.0
    col = G.cyl(13, x_board - (xso - 4 + 2.2 + 0.01), (xso - 4 + 2.2 + 0.01, 0, L.z_el), PX)     # from behind the window seat to the board face
    col = col.cut(G.cbox(3.2, 20.5, 20.5, (x_board - 1.6, 0, L.z_el)))
    col = col.cut(G.box(40, 5, 4, (xso - 6, -2.5, L.z_el + 8)))
    cap = cap.cut(G.cyl(20, 2.2, (xso - 4 - 0.01, 0, L.z_el), X))
    cap = G.fuse(cap, col)
    for dy in (-7, 7):                                   # AS5600 board: 2x M3x8 into nuts slid in from the column side
        x_nut = x_board - 3.2 - 5.0
        od = (0, 1 if dy > 0 else -1, 0)
        cap = hw.clear_hole(cap, 3, (x_board - 3.2, dy, L.z_el + 7), NX, 6)
        cap = hw.nut_trap(cap, 3, (x_nut, dy, L.z_el + 7), NX, od, 8, extra_len=2)
        hw.screw("Head", 3, 8, (x_board - 1.6, dy, L.z_el + 7), NX); hw.nut("Head", 3, (x_nut + NUT[3][1] / 2, dy, L.z_el + 7), NX, open_dir=od)
    _reg(reg, "El_Axis_Cap_L", cap, "Head", "grey", print_up=NX, notes="Left axis cap: AS5600 column (2 mm off the stub magnet), translucent window; 4x M3x12 into bore-loaded nuts.")
    _reg(reg, "Status_Window", G.cyl(19.7, 2, (xso - 4, 0, L.z_el), X), "Head", "clear", print_up=NX, notes="Translucent disc in the left cap.")

    # ---- right lens: translucent disc on the axis over the Stingray-9 channel (its screws are driven through this window)
    lens = G.cyl(r_cov, 7, (xs - 3, 0, z2), X)
    lens = fillet_edges(lens, 2.5, lambda e: e.Curve.TypeId == "Part::GeomCircle" and abs(e.Curve.Radius - r_cov) < 1e-3 and abs(e.Vertexes[0].X - (xs + 4)) < 1e-3, "lens")
    for ang in (45, 135, 225, 315):
        py, pz = r_ls * math.cos(math.radians(ang)), r_ls * math.sin(math.radians(ang))
        seat_pt = (xs + 4 - 3.0, py, z2 + pz)
        lens = hw.cbore_hole(lens, 3, seat_pt, NX, 3.0 + 4, cbore_depth=3.0); hw.screw("Head", 3, 12, seat_pt, NX)
    _reg(reg, "El_Servo_Cover", lens, "Head", "clear", print_up=PX, notes="Translucent 88 mm lens on the right shoulder drum showing the Stingray-9 and its screws; 4x M3x12 into window-loaded nuts.")
    el, _ = placed_stingray("9", el_at, el_x, el_z, "static")
    _reg(reg, "El_Stingray9", el, "Head", "black", printed=False, notes="goBILDA %s, SKU %s: direct EL drive in the right shoulder drum (block + servo + pinion)." % (STINGRAY_KIND["9"]["label"], STINGRAY_KIND["9"]["sku"]))
    elo, _ = placed_stingray("9", el_at, el_x, el_z, "output")
    _reg(reg, "El_Stingray9_Output", elo, "Cradle", "steel", printed=False, notes="Stingray-9 output gear, hub and standoffs: turns with the cradle, bolted to the hub's right wall (4x M4x12 from inside).")



# ============================================================== HEAD (puck, AZ housing, yoke plate)
def build_head(P, L, reg, hw):
    S = STINGRAY
    puck = G.fuse(G.cyl(L.R_lip, L.z_lip1 - L.z_puck0, (0, 0, L.z_puck0)), G.cyl(L.R_body, L.z_deck - L.z_lip1 + 0.01, (0, 0, L.z_lip1 - 0.01)))
    puck = puck.cut(G.tube(L.R_lip + 1, L.R_lip - 1.5, 1.0, (0, 0, L.z_puck0)))
    # Stingray-4 output: 4x M4x25 from the deck into the standoffs (16 mm square), shaft end clearance in the middle
    a = S["standoff"]
    for sx, sy in ((-a, -a), (a, -a), (-a, a), (a, a)):
        puck = hw.cbore_hole(puck, 4, (sx, sy, L.z_deck - 8), DOWN, 30, cbore_depth=8); hw.screw("Head", 4, 25, (sx, sy, L.z_deck - 8), DOWN)
    puck = puck.cut(G.cyl(S["shaft_r"] + 0.5, 2, (0, 0, L.z_puck0 - 0.5)))
    puck = puck.cut(G.cyl(7, 40, (0, -(L.pillar_r - 19), L.z_puck0 - 1)))                       # cable service loop to the base annulus, inside the drum's bottom ring
    pillar_pts = G.pattern_circle(4, L.pillar_r, 0, 45)
    for p in pillar_pts:                                         # pillar feet: M4x30 from below, nuts trapped in the pillar feet
        puck = hw.cbore_hole(puck, 4, (p[0], p[1], L.z_puck0 + 8), UP, 22, cbore_depth=8); hw.screw("Head", 4, 30, (p[0], p[1], L.z_puck0 + 8), UP)
    _reg(reg, "Head_Puck", puck, "Head", "white", notes="Rotating deck: 15 mm journal lip + thrust face (grease); bolted to the Stingray-4 standoffs (4x M4x25). Cable loop hole at the back.")

    # ---- AZ housing: drum aligned with the pedestal, bottom ring on the puck, pillars tied to the wall by ribs
    Rd, wall = L.R_drum, 4.0
    drum = G.tube(Rd, Rd - wall, L.z_drum1 - L.z_deck, (0, 0, L.z_deck))
    drum = G.fuse(drum, G.tube(Rd - wall + 0.01, L.pillar_r - 10, 4, (0, 0, L.z_deck)))                 # bottom ring on the puck
    drum = drum.cut(G.tube(Rd + 1, Rd - 1.2, 1.4, (0, 0, L.z_drum1 - 10)))
    for p in pillar_pts:
        pil = G.cyl(9, L.z_drum1 - L.z_deck, (p[0], p[1], L.z_deck))
        ang = math.degrees(math.atan2(p[1], p[0]))
        rib = G.rot(G.box(Rd - wall - L.pillar_r + 2, 8, L.z_drum1 - L.z_deck, (L.pillar_r - 1, -4, L.z_deck)), Z, ang)
        pil = G.fuse(pil, rib)
        drum = G.fuse(drum, pil)
    for p in pillar_pts:                                         # holes and traps cut after the fuse, or the bottom ring refills the foot hole
        drum = hw.clear_hole(drum, 4, (p[0], p[1], L.z_deck), UP, 12)
        drum = hw.nut_trap(drum, 4, (p[0], p[1], L.z_deck + 9), UP, (-p[0] / L.pillar_r, -p[1] / L.pillar_r, 0), 10, extra_len=3)
        drum = hw.clear_hole(drum, 4, (p[0], p[1], L.z_drum1), DOWN, 12)
        drum = hw.nut_trap(drum, 4, (p[0], p[1], L.z_drum1 - 9), DOWN, (-p[0] / L.pillar_r, -p[1] / L.pillar_r, 0), 10, extra_len=3)
        od = (-p[0] / L.pillar_r, -p[1] / L.pillar_r, 0)
        hw.nut("Head", 4, (p[0], p[1], L.z_deck + 9 - NUT[4][1] / 2), UP, open_dir=od); hw.nut("Head", 4, (p[0], p[1], L.z_drum1 - 9 - NUT[4][1] / 2), UP, open_dir=od)
    drum = drum.cut(G.cbox(14, 10, 8, (0, -Rd, L.z_deck + 22)))
    for i in range(4):
        drum = drum.cut(G.cbox(28, 10, 2, (0, Rd, L.z_deck + 40 + i * 10)))
    drum = G.stencil_text_cut(drum, P["logo_text"], P["logo_font"], 12, 20, at=(0, -Rd + 8, L.z_deck + 55), axis=NY)
    _reg(reg, "Az_Drum", drum, "Head", "white", notes="Rotating azimuth housing (same diameter as the pedestal) = electronics bay. 4 pillars + ribs carry the yoke; M4x30 from below, M4x16 from above, nuts trapped in the pillars.")
    pipe = G.annular_sector(Rd - wall - 2.6, Rd - wall - 0.6, 250, 290, 22, (0, 0, L.z_deck + 44), rounded=False)
    _reg(reg, "Logo_Light_Pipe", pipe, "Head", "clear", notes="Curved translucent plate behind the drum stencil (glue), LED behind.")

    ph, xc = P["yoke_plate_h"], L.x_arm_mid
    ax, by = L.x_sh_out + 4.0, Rd + 2.0                                                          # oval lid: covers the columns in X, the drum in Y
    ell = Part.Ellipse(V(0, 0, L.z_drum1), ax, by)
    plate = Part.Face(Part.Wire(ell.toShape())).extrude(V(0, 0, ph))
    plate = fillet_edges(plate, 4.0, lambda e: e.Curve.TypeId == "Part::GeomEllipse", "plate rim")
    plate = plate.cut(G.cyl(42, ph + 2, (0, 0, L.z_drum1 - 1)))
    for p in pillar_pts:
        plate = hw.cbore_hole(plate, 4, (p[0], p[1], L.z_plate1 - 4.5), DOWN, ph, cbore_depth=4.5); hw.screw("Head", 4, 16, (p[0], p[1], L.z_plate1 - 4.5), DOWN)
    for sgn in (-1, 1):
        for fx in (xc - 18, xc + 18):
            for fy in (-18, 18):
                plate = hw.cbore_hole(plate, 4, (sgn * fx, fy, L.z_drum1 + 4.5), UP, ph, cbore_depth=4.5); hw.screw("Head", 4, 25, (sgn * fx, fy, L.z_drum1 + 4.5), UP)
        plate = plate.cut(G.cyl(8, ph + 2, (sgn * xc, 0, L.z_drum1 - 1)))                       # cables up into the hollow arms
    for pt in G.pattern_circle(6, 50, 0, 0):                     # access cover: M3x25 down into hex pockets on the plate underside
        plate = hw.clear_hole(plate, 3, (pt[0], pt[1], L.z_plate1), DOWN, ph + 1); plate = hw.nut_pocket(plate, 3, (pt[0], pt[1], L.z_drum1), UP, depth=3.0)
        hw.nut("Head", 3, (pt[0], pt[1], L.z_drum1), UP)
    _reg(reg, "Yoke_Plate", plate, "Head", "white", notes="Oval yoke plate / drum lid (244 x 220 mm): pillars 4x M4x16 from above (nuts in the pillars), arm feet 4x M4x25 each from below (nuts in the arm feet), translucent access cover 6x M3x25 (nuts under the plate).")
    acc = G.cyl(58, 4, (0, 0, L.z_plate1))
    acc = fillet_edges(acc, 1.5, lambda e: e.Curve.TypeId == "Part::GeomCircle" and abs(e.Curve.Radius - 58) < 1e-3 and abs(e.Vertexes[0].Z - (L.z_plate1 + 4)) < 1e-3, "access cover")
    acc = G.fuse(acc, G.cyl(41.5, 3, (0, 0, L.z_plate1 - 3)))
    for pt in G.pattern_circle(6, 50, 0, 0):
        acc = hw.cbore_hole(acc, 3, (pt[0], pt[1], L.z_plate1 + 4 - 3), DOWN, 3 + 1, cbore_depth=3); hw.screw("Head", 3, 25, (pt[0], pt[1], L.z_plate1 + 1), DOWN)
    _reg(reg, "Yoke_Access_Cover", acc, "Head", "clear", notes="Translucent round access cover on the yoke plate (electronics visible); 6x M3x25 + nuts under the plate.")


# ============================================================== CRADLE (hollow rounded hub with bolted lid; all nuts inside)
def build_cradle(P, L, reg, hw):
    S = STINGRAY
    hs, zc, bs, tw = L.hub, L.z_el, P["boom_size"], L.hub_wall
    rim, z_rim = 12.0, zc + 27                                                                         # thicker walls at the top hold the lid nuts
    hub = G.cbox(hs, hs, hs, (0, 0, zc))
    hub = fillet_edges(hub, 12.0, lambda e: abs(e.Vertexes[0].Z - e.Vertexes[1].Z) > 10, "hub vertical edges")
    hub = fillet_edges(hub, 3.0, lambda e: abs(e.Vertexes[0].Z - e.Vertexes[1].Z) < 1e-3, "hub top/bottom edges")
    hub = hub.cut(G.cbox(hs - 2 * tw, hs - 2 * tw, z_rim - (zc - hs / 2 + tw), (0, 0, (z_rim + zc - hs / 2 + tw) / 2)))     # cavity
    hub = hub.cut(G.cbox(hs - 2 * rim, hs - 2 * rim, hs, (0, 0, z_rim + hs / 2)))                      # narrower opening through the rim
    hub = hub.cut(G.cbox(hs - 2 * tw + 7, hs - 2 * tw + 7, 6, (0, 0, zc + hs / 2 - 3)))                # lid rebate (81 sq x 6)
    for sgn in (1, -1):                                                                                # boom / cw flanges: M4x20 into nuts inside
        hub = hub.cut(G.cyl(15.3, 10.3, (0, sgn * (hs / 2 + 0.01), zc), (0, -sgn, 0)))              # round spigot recess
        for (dx, dz) in ((-21, -21), (21, -21), (-21, 21), (21, 21)):
            p = (dx, sgn * hs / 2, zc + dz)
            hub = hw.clear_hole(hub, 4, p, (0, -sgn, 0), tw + 1); hw.nut("Cradle", 4, (dx, sgn * (hs / 2 - tw), zc + dz), (0, -sgn, 0))
    # left stub flange: spigot recess + M4x20 into nuts inside
    hub = hub.cut(G.cyl(15.3, 6.3, (-(hs / 2 + 0.01), 0, zc), PX))
    for p in G.pattern_circle(4, 20, 0, 45):
        q = (-hs / 2, p[0], zc + p[1])
        hub = hw.clear_hole(hub, 4, q, PX, tw + 1); hw.nut("Cradle", 4, (-(hs / 2 - tw), p[0], zc + p[1]), PX)
    # right wall: bolted to the Stingray-9 standoffs, 4x M4x12 from inside the cavity into the gearbox's own M4 threads
    a = S["standoff"]
    for dy, dz in ((-a, -a), (a, -a), (-a, a), (a, a)):
        hub = hw.clear_hole(hub, 4, (hs / 2, dy, zc + dz), NX, tw + 1); hw.screw("Cradle", 4, 12, (hs / 2 - tw, dy, zc + dz), PX)
    hub = hub.cut(G.cyl(S["shaft_r"] + 0.5, 2, (hs / 2 - 1.5, 0, zc), PX))                              # shaft end clearance
    hub = hub.cut(G.cyl(5, tw + 2, (0, 0, zc - hs / 2 - 1)))                                             # coax through the floor
    # lid bolts: 4x M3x12 down through the lid into nuts side-loaded from inside the rim
    r_lid = hs / 2 - 8.5                                                                             # heads stay inside the lid, nuts in the rim
    for (dx, dy) in ((-r_lid, 0), (r_lid, 0), (0, -r_lid), (0, r_lid)):
        od = (-dx / r_lid, -dy / r_lid, 0)
        hub = hw.clear_hole(hub, 3, (dx, dy, zc + hs / 2), DOWN, 12)
        hub = hw.nut_trap(hub, 3, (dx, dy, zc + hs / 2 - 12), DOWN, od, 8, extra_len=4)
        hw.nut("Cradle", 3, (dx, dy, zc + hs / 2 - 12 - NUT[3][1] / 2), UP, open_dir=od)
    _reg(reg, "Cradle_Hub", hub, "Cradle", "black", notes="Hollow rounded hub, open top with a flush bolted lid: every flange bolt (left stub, boom, counterweight arm) ends in a nut inside the cavity; the right wall bolts straight onto the Stingray-9 standoffs.")
    lid = G.cbox(hs - 2 * tw + 6.4, hs - 2 * tw + 6.4, 6, (0, 0, zc + hs / 2 - 3))
    lid = lid.cut(G.cyl(5, 12, (0, 0, zc + hs / 2 - 5)))
    for (dx, dy) in ((-r_lid, 0), (r_lid, 0), (0, -r_lid), (0, r_lid)):
        lid = hw.cbore_hole(lid, 3, (dx, dy, zc + hs / 2 - 3), DOWN, 8, cbore_depth=3.0); hw.screw("Cradle", 3, 12, (dx, dy, zc + hs / 2 - 3), DOWN)
    _reg(reg, "Cradle_Lid", lid, "Cradle", "black", print_up=DOWN, notes="Flush hub lid, 4x M3x12 into nuts trapped in the rim. Fit after all flange nuts and the standoff screws are in.")

    xf, x_end = -hs / 2, -L.x_stub_end
    stub = G.cyl(L.r_axle, abs(x_end - xf), (min(xf, x_end), 0, zc), X)
    fl = G.cyl(30, 6, (xf, 0, zc), NX)
    stub = G.fuse(stub, fl, G.cyl(15, 6, (xf, 0, zc), PX))
    for p in G.pattern_circle(4, 20, 0, 45):
        seat = (xf - 6 + 4.5, p[0], zc + p[1])
        stub = hw.cbore_hole(stub, 4, seat, PX, 4.5 + 1 + 6, cbore_depth=4.5); hw.screw("Cradle", 4, 20, seat, PX)
    stub = stub.cut(G.cyl(3.1, 3, (x_end + 3, 0, zc), NX))
    _reg(reg, "El_Stub_L", stub, "Cradle", "black", print_up=PX, notes="Left EL stub axle, flange 4x M4x20 through the hub wall into nuts inside; turns in the split bushings. Magnet in the end face.")

    def round_arm(y_root_sign, y_end, name, color, notes, extra=None):
        yr = y_root_sign * hs / 2; length = abs(y_end - yr); d = (0, y_root_sign, 0)
        tube_ = G.cyl(bs / 2, length, (0, yr, zc), d).cut(G.cyl(bs / 2 - 4, length + 2, (0, yr - y_root_sign, zc), d))
        tube_ = G.fuse(tube_, G.cyl(bs / 2, 12, (0, yr, zc), d), G.cyl(15, 10, (0, yr, zc), (0, -y_root_sign, 0)))   # root plug + spigot
        tube_ = G.fuse(tube_, G.cyl(35, 6, (0, yr, zc), d))                                                           # round root flange
        for (dx, dz) in ((-21, -21), (21, -21), (-21, 21), (21, 21)):
            seat = (dx, yr + y_root_sign * 1.5, zc + dz)
            tube_ = hw.cbore_hole(tube_, 4, seat, (0, -y_root_sign, 0), 2, cbore_depth=4.5); hw.screw("Cradle", 4, 20, seat, (0, -y_root_sign, 0))
        if extra: tube_ = extra(tube_)
        _reg(reg, name, tube_, "Cradle", color, notes=notes)

    def boom_end(tb):
        fl = G.fuse(G.cyl(32, 8, (0, L.y_boom_end - 8, zc), Y), G.cyl(bs / 2, 4, (0, L.y_boom_end - 12, zc), Y))
        for p in G.pattern_circle(4, L.r_adapter_bolt, 0, 0):       # M5 nuts in blind hex pockets on the back of the flange, clear of the tube
            fl = hw.clear_hole(fl, 5, (p[0], L.y_boom_end - 8, zc + p[1]), PY, 8)
            fl = hw.nut_pocket(fl, 5, (p[0], L.y_boom_end - 8, zc + p[1]), PY, depth=5.0)
        return G.fuse(tb, fl).cut(G.cyl(6, 16, (0, L.y_boom_end - 14, zc), Y))   # coax hole through the flange and its backing into the tube
    round_arm(1, L.y_boom_end - 8, "Dish_Boom", "black", "Round boom tube to the dish offset; round root flange 4x M4x20 into nuts inside the hub; round end flange takes either adapter (4x M5 + nuts).", boom_end)

    def cw_holes(tb):
        for i in range(6):
            tb = hw.clear_hole(tb, 4, (-bs / 2 - 1, L.y_cw_end + 20 + i * 22, zc), PX, bs + 2)
        return tb
    round_arm(-1, L.y_cw_end, "Counterweight_Arm", "black", "Round tube with a row of M4 holes for the canister collar; round root flange 4x M4x20 into nuts inside the hub.", cw_holes)

    ad = P["adapter_flange_d"]
    plate = G.cyl(ad / 2, 8, (0, L.y_boom_end, zc), Y)
    plate = fillet_edges(plate, 2.0, lambda e: e.Curve.TypeId == "Part::GeomCircle" and abs(e.Curve.Radius - ad / 2) < 1e-3, "adapter plate")
    for ang in range(0, 360, 45):
        plate = plate.cut(G.rot(G.cbox(7, 10, 24, (0, L.y_boom_end + 4, zc + 46)), Y, ang, (0, 0, zc)))
    for ang in range(0, 360, 45):
        w_ = G.rot(G.cbox(4, 10, 40, (0, L.y_boom_end + 4, zc + 28)), Y, ang + 22.5, (0, 0, zc))
        plate = plate.cut(w_.common(G.tube(ad / 2 - 6, 30, 12, (0, L.y_boom_end - 1, zc), Y)))
    plate = plate.cut(G.cyl(6, 10, (0, L.y_boom_end - 1, zc), Y))
    for p in G.pattern_circle(4, L.r_adapter_bolt, 0, 0):          # M5x12 into the nuts pocketed in the boom flange
        plate = hw.cbore_hole(plate, 5, (p[0], L.y_boom_end + 8 - 5.5, zc + p[1]), NY, 16, cbore_depth=5.5)
        hw.screw("Cradle", 5, 12, (p[0], L.y_boom_end + 8 - 5.5, zc + p[1]), NY); hw.nut("Cradle", 5, (p[0], L.y_boom_end - 8 + 5, zc + p[1]), NY)
    _reg(reg, "Dish_Adapter_Plate", plate, "Cradle", "grey", print_up=PY, notes="Adapter A: slotted plate; 4x M5x12 into nuts pocketed in the boom flange.")
    stub = G.cyl(32, 8, (0, L.y_boom_end, zc), Y)
    for p in G.pattern_circle(4, L.r_adapter_bolt, 0, 0):
        stub = hw.cbore_hole(stub, 5, (p[0], L.y_boom_end + 8 - 5.5, zc + p[1]), NY, 16, cbore_depth=5.5)
    stub = G.fuse(stub, G.tube(24.15, 20, 100, (0, L.y_boom_end + 8, zc), Y))
    _reg(reg, "Dish_Adapter_PipeStub", stub, "Cradle", "grey", print_up=PY, hidden=True, notes="Adapter B: 48.3 mm pipe stub.")

    cd, cl = P["cw_canister_d"], P["cw_canister_len"]; yc0 = L.y_cw_end
    can = G.cyl(cd / 2, cl, (0, yc0, zc), NY).cut(G.cyl(cd / 2 - 2.5, cl - 4, (0, yc0 - 4, zc), NY))
    collar = G.cyl(bs / 2 + 4, 60, (0, yc0 + 56, zc), NY)
    collar = fillet_edges(collar, 3.0, lambda e: e.Curve.TypeId == "Part::GeomCircle" and abs(e.Curve.Radius - (bs / 2 + 4)) < 1e-3 and abs(e.Vertexes[0].Y - (yc0 + 56)) < 1e-3, "collar")
    collar = collar.cut(G.cyl(bs / 2 + 0.25, 62, (0, yc0 + 57, zc), NY))
    can = G.fuse(can, collar)
    for yy in (yc0 + 20, yc0 + 42):
        can = hw.clear_hole(can, 4, (-bs / 2 - 5, yy, zc), PX, bs + 10); hw.screw("Cradle", 4, 55, (-bs / 2 - 4, yy, zc), PX); hw.nut("Cradle", 4, (bs / 2 + 4, yy, zc), PX)
    y_cap, y_bolt = yc0 - cl + 8, yc0 - cl + 4                     # cap skirt covers the last 8 mm of the canister; bolts mid-skirt
    r_plug = cd / 2 - 2.5 - 0.3
    for ang in (0, 180):                                           # cap bolts: M3x10 through the cap skirt and canister wall, nuts pocketed in the cap's plug
        c_, s_ = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        can = hw.clear_hole(can, 3, ((cd / 2 + 2) * c_, y_bolt, zc + (cd / 2 + 2) * s_), (-c_, 0, -s_), 6)
        hw.nut("Cradle", 3, (r_plug * c_, y_bolt, zc + r_plug * s_), (-c_, 0, -s_))
    _reg(reg, "Counterweight_Canister", can, "Cradle", "black", print_up=PY, notes="Opaque canister (~1.3 L); collar 2x M4x55 + nuts; cap 2x M3x10 with nuts inside the rim.")
    cap = G.cyl(cd / 2 + 2, 10, (0, y_cap, zc), NY)
    cap = fillet_edges(cap, 3.0, lambda e: e.Curve.TypeId == "Part::GeomCircle" and abs(e.Curve.Radius - (cd / 2 + 2)) < 1e-3 and abs(e.Vertexes[0].Y - (y_cap - 10)) < 1e-3, "cw cap")
    cap = cap.cut(G.cyl(cd / 2 + 0.25, 8, (0, y_cap, zc), NY))                                         # skirt + 2 mm end disc
    cap = G.fuse(cap, G.cyl(r_plug, 8.01, (0, y_cap, zc), NY))                                         # plug inside the canister bore
    cap = cap.cut(G.text_solid(P["logo_text"], P["logo_font"], 12, 1.0, at=(0, yc0 - cl - 1, zc), axis=NY))
    for ang in (0, 180):                                           # radial hex pockets in the plug: the nut drops in, the canister wall then closes it
        c_, s_ = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        p = ((cd / 2 + 2) * c_, y_bolt, zc + (cd / 2 + 2) * s_); d = (-c_, 0, -s_)
        cap = hw.clear_hole(cap, 3, p, d, 13)
        cap = hw.nut_pocket(cap, 3, (r_plug * c_, y_bolt, zc + r_plug * s_), d, depth=2.7)
        hw.screw("Cradle", 3, 10, p, d)
    _reg(reg, "Counterweight_Cap", cap, "Cradle", "orange", print_up=PY, notes="Press cap, engraved PERIGEE; 2x M3x10 into nuts pocketed in the plug.")

    D, f, t = P["dish_diameter"], P["dish_focal_length"], 3.0
    pts = [V(D / 2 * i / 24, (D / 2 * i / 24) ** 2 / (4 * f), 0) for i in range(25)]
    outer = Part.BSplineCurve(); outer.interpolate(pts)
    inner = Part.BSplineCurve(); inner.interpolate([V(p.x, p.y + t, 0) for p in pts])
    wire = Part.Wire([outer.toShape(), Part.makeLine(pts[-1], V(pts[-1].x, pts[-1].y + t, 0)), inner.toShape(), Part.makeLine(V(pts[0].x, pts[0].y + t, 0), pts[0])])
    dish = Part.Face(wire).revolve(V(0, 0, 0), V(0, 1, 0), 360)
    dish.Placement = App.Placement(V(0, L.y_boom_end + 8, zc), App.Rotation())
    _reg(reg, "Dish_Reference_1m", dish.removeSplitter(), "Cradle", "ref", printed=False, notes="Reference paraboloid, not printed.")
    feed = G.fuse(G.cyl(4, f - 40, (0, L.y_boom_end + 28, zc + 30), Y), G.cyl(4, f - 40, (0, L.y_boom_end + 28, zc - 30), Y), G.cyl(20, 40, (0, L.y_boom_end + 8 + f - 20, zc), Y))
    _reg(reg, "Feed_Reference", feed, "Cradle", "ref", printed=False, notes="LNB / feed placeholder.")


def build_markers(P, L, reg):
    zc = L.z_el
    def mk(name, at, axis, group):
        _reg(reg, name, G.cyl(1.0, 0.5, at, axis), group, "steel", printed=False, hidden=True)
    mk("_JM_Az_Base", (0, 0, L.z_puck0 - 0.75), Z, "Base"); mk("_JM_Az_Head", (0, 0, L.z_puck0 - 0.75), Z, "Head")
    mk("_JM_El_Head", (0, 0, zc), X, "Head"); mk("_JM_El_Cradle", (0, 0, zc), X, "Cradle")


def build_all(P):
    L = Layout(P); reg = {}; hw = Hardware()
    build_base(P, L, reg, hw); build_arms(P, L, reg, hw); build_head(P, L, reg, hw)
    build_cradle(P, L, reg, hw); build_markers(P, L, reg)
    return L, reg, hw
