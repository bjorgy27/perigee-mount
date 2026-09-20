"""Part builders for the Perigee gimbal (v6: CMG-ISS proportions, printed plain bearings, bolts + captive nuts only).

World coordinates at the nominal pose (az = 0, el = 0, boresight +Y, az axis Z, el axis X).
Fastening rule: no threads in plastic. Every bolt ends in a hex nut, either in a blind hex pocket on the far face
(when that face is reachable at assembly time) or in a side-loaded nut trap.
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


def fillet_edges(shape, radius, pick):
    try:
        edges = [e for e in shape.Edges if pick(e)]
        if not edges:
            return shape
        f = shape.makeFillet(radius, edges)
        return f if f.isValid() and len(f.Solids) == 1 else shape
    except Exception:
        return shape


def side_name(sgn):
    return "R" if sgn > 0 else "L"


class Layout:
    def __init__(self, P):
        self.P = P
        m = P["gear_module"]; self.m = m
        self.r_ring = G.pitch_r(P["az_ring_teeth"], m); self.r_azpin = G.pitch_r(P["az_pinion_teeth"], m)
        self.az_pinion_c = (0.0, self.r_ring - self.r_azpin)
        self.r_elgear = G.pitch_r(P["el_gear_teeth"], m); self.r_elpin = G.pitch_r(P["el_pinion_teeth"], m)
        sm = P["sensor_module"]
        self.r_sun = G.pitch_r(P["sensor_sun_teeth"], sm); self.r_idler = G.pitch_r(P["sensor_idler_teeth"], sm); self.r_sens = G.pitch_r(P["sensor_gear_teeth"], sm)
        self.idler_c = (0.0, -(self.r_sun + self.r_idler)); self.sens_c = (0.0, -(self.r_sun + 2 * self.r_idler + self.r_sens))
        self.fw = P["gear_face_width"]
        self.z_floor = P["base_floor"]; self.z_ring0 = 17.0; self.z_ring1 = self.z_ring0 + self.fw
        self.z_pin_hub1 = 39.0; self.z_sens0, self.z_sens1 = 31.0, 39.0; self.z_tower_top = 34.0
        self.z_puck0 = P["puck_z0"]; self.z_deck = self.z_puck0 + P["puck_h"]
        self.R_lip = P["puck_od"] / 2; self.R_body = P["puck_body_od"] / 2; self.z_lip1 = self.z_puck0 + P["puck_lip_h"]
        self.R_base = P["base_od"] / 2; self.R_pedestal = P["base_body_od"] / 2; self.R_bore = P["base_wall_inner_r"]; self.R_thrust = P["thrust_ring_r"]
        self.z_base_top = P["base_h"]; self.z_ret1 = self.z_base_top + P["retainer_h"]
        self.R_drum = P["drum_od"] / 2; self.z_drum1 = self.z_deck + P["drum_h"]
        self.z_plate1 = self.z_drum1 + P["yoke_plate_h"]
        self.z_el = self.z_deck + P["el_axis_height"]
        self.x_arm_in = P["arm_inner_x"]; self.x_arm_out = self.x_arm_in + P["arm_thick"]; self.x_arm_mid = (self.x_arm_in + self.x_arm_out) / 2
        self.r_axle = P["el_axle_d"] / 2; self.r_bush = P["el_bushing_od"] / 2
        self.R_sh = P["shoulder_r"]; self.x_sh_out = self.x_arm_in + P["shoulder_thick"]
        self.arm_splits = [self.z_plate1, self.z_el - (P["segment_max"] - 6), self.z_el]
        self.el_pin_rho = self.r_elgear + self.r_elpin; self.z_elpin = self.z_el - self.el_pin_rho
        self.hub = P["hub_size"]; self.hub_wall = P["hub_wall"]
        self.x_gear0 = self.hub / 2 + 1; self.x_gear1 = self.x_gear0 + 10
        self.x_servo0 = self.x_arm_in + 4
        self.x_stub_end = self.x_sh_out - 12
        self.y_boom_end = P["dish_offset"]; self.y_cw_end = -(self.hub / 2 + P["cw_arm_len"])
        self.pillar_r = self.R_body - 12.5                     # pillars stand on the puck body, tied to the drum wall by ribs

    def arm_halfwidth(self, z):
        P = self.P
        s = min(1.0, max(0.0, (z - self.z_plate1) / (self.z_el - self.z_plate1)))
        lin = P["arm_max_halfwidth"] - (P["arm_max_halfwidth"] - P["arm_top_halfwidth"]) * s
        rho = self.z_el - z
        return min(lin, max(P["arm_top_halfwidth"], math.tan(math.radians(P["arm_cone_deg"])) * rho))


def _reg(reg, name, shape, group, color, qty=1, print_up=(0, 0, 1), notes="", printed=True, hidden=False):
    reg[name] = dict(shape=shape, group=group, color=color, qty=qty, print_up=print_up, notes=notes, printed=printed, hidden=hidden)
    return shape


# ============================================================== BASE
def build_base(P, L, reg, hw):
    base = G.fuse(G.cyl(L.R_base, 10), G.cyl(L.R_pedestal, L.z_base_top))
    base = fillet_edges(base, 3.0, lambda e: e.Curve.TypeId == "Part::GeomCircle" and abs(e.Curve.Radius - L.R_base) < 1e-3 and abs(e.Vertexes[0].Z - 10) < 1e-3)
    base = base.cut(G.cyl(L.R_thrust, L.z_base_top, (0, 0, L.z_floor)))
    base = base.cut(G.cyl(L.R_bore, L.z_base_top, (0, 0, L.z_puck0)))
    for ang in (30, 150, 270):
        base = base.cut(G.rot(G.cbox(6, 22, 14, (L.R_pedestal - 1.5, 0, 26)), Z, ang))
    tower = G.fuse(G.cyl(25, 27 - L.z_floor, (0, 0, L.z_floor)), G.cyl(P["tower_od"] / 2, L.z_tower_top - 27, (0, 0, 27)))
    tower = tower.cut(G.box(20, 60, 13, (P["tower_od"] / 2 - 1.5, -30, 27)))
    base = G.fuse(base, tower)
    base = base.cut(G.cyl(P["slipring_body_d"] / 2 + 0.3, 30, (0, 0, L.z_tower_top - 2 - P["slipring_body_len"])))
    base = base.cut(G.cyl(15.8, 3, (0, 0, L.z_tower_top - 2)))
    base = base.cut(G.cyl(6, 20, (0, 0, L.z_floor - 0.01)))
    base = base.cut(G.box(10, L.R_base + 5, 8, (-5, -(L.R_base + 5), L.z_floor)))
    # slip ring flange: 2x M3 down through the flange into nuts side-loaded from the tower shoulder (z 27) region
    for ang in (90, 270):
        p = (13.0 * math.cos(math.radians(ang)), 13.0 * math.sin(math.radians(ang)), L.z_tower_top - 2)
        base = hw.clear_hole(base, 3, p, DOWN, 8)
        base = hw.nut_trap(base, 3, (p[0], p[1], L.z_tower_top - 2 - 5.5), DOWN, (0, math.sin(math.radians(ang)), 0), 10)
        hw.screw("Base", 3, 10, (p[0], p[1], L.z_tower_top), DOWN); hw.nut("Base", 3, (p[0], p[1], L.z_tower_top - 2 - 5.5 - NUT[3][1] / 2), DOWN)
    # ring gear: 6x M4 up from under the floor into nuts side-loaded into the ring's plain section
    for p in G.pattern_circle(6, L.r_ring + 5.5, 0, 30):
        base = hw.cbore_hole(base, 4, (p[0], p[1], 5), UP, 1.5, cbore_depth=5); hw.screw("Base", 4, 12, (p[0], p[1], 5), UP)
    # retainer flange plate: 12x M4 down into nut traps opening on the pedestal's outer face
    for p in G.pattern_circle(P["retainer_bolts"], 110, L.z_base_top, 15):
        base = hw.clear_hole(base, 4, p, DOWN, 12)
        rd = (p[0] / 110.0, p[1] / 110.0, 0)
        base = hw.nut_trap(base, 4, (p[0], p[1], L.z_base_top - 9), DOWN, rd, 10)
        hw.nut("Base", 4, (p[0], p[1], L.z_base_top - 9 - NUT[4][1] / 2), DOWN)
    for p in G.pattern_circle(6, 118, 0, 30):
        base = base.cut(G.cyl(3.3, 12, p))
    for p in G.pattern_circle(3, 80, 0, 15):
        base = base.cut(G.cyl(2, L.z_floor + 1, p))
    _reg(reg, "Base_Cup", base, "Base", "black", notes="AZ pedestal. Step at z 40 (r 92-104) is the greased thrust face; 208 mm bore is the journal. 6x M6 in the foot flange for a mount plate. Nut traps for the flange-plate bolts open on the outside.")

    ret = G.tube(L.R_base, L.R_body + 0.5, P["retainer_h"], (0, 0, L.z_base_top))
    ret = fillet_edges(ret, 2.5, lambda e: e.Curve.TypeId == "Part::GeomCircle" and abs(e.Curve.Radius - L.R_base) < 1e-3 and abs(e.Vertexes[0].Z - L.z_ret1) < 1e-3)
    for p in G.pattern_circle(P["retainer_bolts"], 110, L.z_ret1, 15):
        ret = hw.cbore_hole(ret, 4, (p[0], p[1], L.z_ret1 - 3.5), DOWN, P["retainer_h"], cbore_depth=3.5); hw.screw("Base", 4, 20, (p[0], p[1], L.z_ret1 - 3.5), DOWN)
    _reg(reg, "Az_Retainer_Ring", ret, "Base", "black", notes="Wide flange plate (12x M4x20 + nuts in the pedestal wall) trapping the puck lip: uplift + moment, and the visible AZ seam.")

    ring = G.internal_ring_gear(P["az_ring_teeth"], L.m, L.fw, L.R_thrust - 3.5, backlash=P["gear_backlash"], at=(0, 0, L.z_ring0))
    ring = G.fuse(ring, G.tube(L.R_thrust - 3.5, L.r_ring + 2.6, L.z_ring0 - L.z_floor, (0, 0, L.z_floor)))
    for p in G.pattern_circle(6, L.r_ring + 5.5, L.z_floor, 30):
        ring = hw.clear_hole(ring, 4, p, UP, 6)
        rd = (-p[0] / (L.r_ring + 5.5), -p[1] / (L.r_ring + 5.5), 0)                   # slot opens toward the inside face
        ring = hw.nut_trap(ring, 4, (p[0], p[1], L.z_floor + 5.5), UP, rd, 8, extra_len=4)
        hw.nut("Base", 4, (p[0], p[1], L.z_floor + 5.5 - NUT[4][1] / 2), UP)
    _reg(reg, "Az_Ring_Gear", ring, "Base", "orange", notes="Internal ring gear %d T module %g; 6x M4x12 from below, nuts slid into the inner face slots." % (P["az_ring_teeth"], L.m))
    sun = G.spur_gear(P["sensor_sun_teeth"], P["sensor_module"], 12, backlash=P["gear_backlash"], at=(0, 0, 27))
    sun = sun.cut(G.cyl(P["tower_od"] / 2 + 0.2, 13, (0, 0, 26.5)).cut(G.box(20, 60, 14, (P["tower_od"] / 2 - 1.5 + 0.2, -30, 26))))
    _reg(reg, "Sensor_Sun_Gear", sun, "Base", "orange", notes="Fixed sun of the AZ encoder train: D-flat press fit, a drop of CA; the puck retains it axially.")
    cap = G.fuse(G.cyl(P["slipring_body_d"] / 2, P["slipring_body_len"], (0, 0, L.z_tower_top - 2 - P["slipring_body_len"])),
                 G.cyl(15.5, 2, (0, 0, L.z_tower_top - 2)), G.cyl(P["slipring_stub_d"] / 2, 16, (0, 0, L.z_tower_top)))
    for ang in (90, 270):
        cap = cap.cut(G.cyl(1.7, 5, (13.0 * math.cos(math.radians(ang)), 13.0 * math.sin(math.radians(ang)), L.z_tower_top - 3)))
    _reg(reg, "SlipRing_Capsule_22mm", cap, "Base", "steel", printed=False, notes="22 mm flanged capsule slip ring (bought).")


# ============================================================== ARMS
def arm_profile(L, z0, z1, steps=30):
    zs = [z0 + (z1 - z0) * i / steps for i in range(steps + 1)]
    return [(L.arm_halfwidth(z), z) for z in zs] + [(-L.arm_halfwidth(z), z) for z in reversed(zs)]


def yz_prism(pts_yz, x0, x1):
    pts = [V(x0, y, z) for y, z in pts_yz] + [V(x0, pts_yz[0][0], pts_yz[0][1])]
    return Part.Face(Part.makePolygon(pts)).extrude(V(x1 - x0, 0, 0))


def build_arms(P, L, reg, hw):
    x0, x1, xs = L.x_arm_in, L.x_arm_out, L.x_sh_out
    z0, z1, z2 = L.arm_splits
    Rsh = L.R_sh
    w, h, ln = P["servo_w"], P["servo_h"], P["servo_len"]

    low = yz_prism(arm_profile(L, z0, z1), x0, x1)
    low = fillet_edges(low, 8.0, lambda e: abs(e.Vertexes[0].Z - e.Vertexes[1].Z) > 20 and min(e.Vertexes[0].Z, e.Vertexes[1].Z) > z0 - 1 and max(e.Vertexes[0].Z, e.Vertexes[1].Z) < z1 + 1)
    zp0, zp1 = z0 + 16, z1 - 34
    wmin = min(L.arm_halfwidth(zp0), L.arm_halfwidth(zp1)) - 12
    low = low.cut(G.cbox(3, 2 * wmin, zp1 - zp0, (x1, 0, (zp0 + zp1) / 2)))
    low = low.cut(G.cbox(6, 6, z1 - z0 + 2, (x0, -L.arm_halfwidth(z1) + 9, (z0 + z1) / 2)))
    w_m = L.arm_halfwidth(z1) - 12
    low = low.cut(G.cbox(14.5, 2 * w_m + 0.5, 25.5, (L.x_arm_mid, 0, z1 - 12.5)))
    for yy in (-0.7 * w_m, 0.7 * w_m):
        low = hw.cbore_hole(low, 4, (x1 - 4.5, yy, z1 - 12), NX, 40, cbore_depth=4.5)
        low = hw.nut_pocket(low, 4, (x0, yy, z1 - 12), PX, depth=3.6)
    # foot bolts come up from under the yoke plate; nuts slide into traps from the arm's faces at z0 + 9
    for fx, od in ((x0 + 6, NX), (x1 - 6, PX)):
        for fy in (-40, 40):
            low = hw.clear_hole(low, 4, (fx, fy, z0), UP, 7)
            low = hw.nut_trap(low, 4, (fx, fy, z0 + 9), UP, od, 8, extra_len=3)

    up = yz_prism(arm_profile(L, z1, z2), x0, x1)
    sh_prof = [(Rsh, z2), (Rsh, z2 - 110), (L.arm_halfwidth(z2 - 150), z2 - 150), (-L.arm_halfwidth(z2 - 150), z2 - 150), (-Rsh, z2 - 110), (-Rsh, z2)]
    shoulder = yz_prism(sh_prof, x0, xs)
    shoulder = fillet_edges(shoulder, 14.0, lambda e: abs(e.Vertexes[0].Z - e.Vertexes[1].Z) > 30 and abs(abs(e.Vertexes[0].Y) - Rsh) < 1e-3)
    up = G.fuse(up, shoulder)
    up = up.cut(G.rot(G.cbox(40, 200, 40, (xs, 0, z2 - 150)), Y, 45, (xs, 0, z2 - 150)).common(G.box(60, 200, 60, (x1, -100, z2 - 200))))
    up = G.fuse(up, G.cbox(14, 2 * w_m, 25, (L.x_arm_mid, 0, z1 - 12.5)))
    for yy in (-0.7 * w_m, 0.7 * w_m):
        up = hw.clear_hole(up, 4, (x1 + 1, yy, z1 - 12), NX, 40)
    up = up.cut(G.cbox(6, 6, z2 - z1 + 2, (x0, -L.arm_halfwidth(z2 - 150) + 9, (z1 + z2 - 150) / 2)))
    up = up.cut(G.cyl(L.r_bush + 0.2, xs - x0 + 2, (x0 - 1, 0, L.z_el), X))
    # pillow cap bolts: down from the cap into nut traps that open on the shoulder's +/-Y faces, 10 mm below the top
    for yy in (-44, 44):
        up = hw.clear_hole(up, 4, ((x0 + xs) / 2, yy, z2), DOWN, 12)
        up = hw.nut_trap(up, 4, ((x0 + xs) / 2, yy, z2 - 10), DOWN, (0, 1 if yy > 0 else -1, 0), 24, extra_len=4)
    # on-axis cap seat + 4x M3 in the lower half, nuts slid in radially from the bore before the bushing goes in
    up = up.cut(G.cyl(40.2, 3, (xs - 3, 0, L.z_el), X))
    for ang in (205, 245, 295, 335):
        py, pz = 33 * math.cos(math.radians(ang)), 33 * math.sin(math.radians(ang))
        up = hw.clear_hole(up, 3, (xs - 3, py, L.z_el + pz), NX, 7)
        up = hw.nut_trap(up, 3, (xs - 8, py, L.z_el + pz), NX, (0, -py / 33, -pz / 33), 33 - L.r_bush - 0.5, extra_len=3)
    for side, sgn in (("R", 1), ("L", -1)):
        lo = low if sgn > 0 else low.mirror(V(0, 0, 0), V(1, 0, 0))
        zmid = (zp0 + zp1) / 2
        mp = None if sgn > 0 else ((0, 0, zmid), (0, 1, 0))
        lo = G.stencil_text_cut(lo, P["logo_text"], P["logo_font"], P["logo_size"], 60, at=((x0 - 5) if sgn > 0 else -(x1 + 5), 0, zmid), axis=X, rotate_deg=180, mirror_plane=mp)
        for fx in (x0 + 6, x1 - 6):
            for fy in (-40, 40):
                hw.nut("Head", 4, (sgn * fx, fy, z0 + 9 - NUT[4][1] / 2), UP)
        for yy in (-0.7 * w_m, 0.7 * w_m):
            hw.screw("Head", 4, 40, (sgn * (x1 - 4.5), yy, z1 - 12), (-sgn, 0, 0)); hw.nut("Head", 4, (sgn * x0, yy, z1 - 12), (sgn, 0, 0))
        _reg(reg, "Arm_%s_1" % side, lo, "Head", "white", print_up=(-sgn, 0, 0), notes="Lower yoke arm (%s). Foot bolts (4x M4x25) come up from under the yoke plate into side-loaded nuts; tenon + 2x M4x40 to the shoulder." % side)

        u = up if sgn > 0 else up.mirror(V(0, 0, 0), V(1, 0, 0))
        xso = sgn * xs
        if sgn > 0:
            u = u.cut(G.cbox(xs - L.x_servo0 + 1, w + 1, h + 1, ((xs + L.x_servo0 + 1) / 2, 0, L.z_elpin)))
            u = u.cut(G.cyl(7.5, x0 + 10, (x0 - 1, 0, L.z_elpin), X))
            u = u.cut(G.cyl(46.2, 3, (xs - 3, 0, L.z_elpin), X))
            for ang in (45, 135, 225, 315):                       # servo cover: 4x M3, nuts slid in from the servo pocket walls
                py, pz = 40 * math.cos(math.radians(ang)), 40 * math.sin(math.radians(ang))
                u = hw.clear_hole(u, 3, (xs - 3, py, L.z_elpin + pz), NX, 7)
                u = hw.nut_trap(u, 3, (xs - 8, py, L.z_elpin + pz), NX, (0, -py / 40, -pz / 40), 40 - (w / 2 + 0.5) + 2, extra_len=3)
                hw.nut("Head", 3, (xs - 8 + NUT[3][1] / 2, py, L.z_elpin + pz), PX)
        for yy in (-44, 44):
            hw.nut("Head", 4, (sgn * (x0 + xs) / 2, yy, z2 - 10 - NUT[4][1] / 2), UP)
        for ang in (205, 245, 295, 335):
            py, pz = 33 * math.cos(math.radians(ang)), 33 * math.sin(math.radians(ang))
            hw.nut("Head", 3, (sgn * (xs - 8) + sgn * NUT[3][1] / 2 * 0, py, L.z_el + pz), (sgn, 0, 0))
        _reg(reg, "Arm_%s_2" % side, u, "Head", "white", print_up=(-sgn, 0, 0),
             notes="Shoulder segment (%s): pillow block top, on-axis cap%s. All nuts side-loaded (traps on the Y faces, from the bore, from the servo pocket)." % (side, ", servo pocket behind the lower cover" if sgn > 0 else ""))

    for sgn in (1, -1):
        xa, xb = min(sgn * x0, sgn * xs), max(sgn * x0, sgn * xs)
        cap = G.cyl(Rsh, xb - xa, (xa, 0, L.z_el), X).common(G.box(xb - xa, 2 * Rsh, Rsh + 1, (xa, -Rsh, L.z_el)))
        cap = fillet_edges(cap, 14.0, lambda e: e.Curve.TypeId == "Part::GeomCircle" and (abs(abs(e.Curve.Center.x) - xb) < 1e-3 if sgn > 0 else abs(abs(e.Curve.Center.x) - xa) < 1e-3))
        cap = cap.cut(G.cyl(L.r_bush + 0.2, xb - xa + 2, (xa - 1, 0, L.z_el), X))
        for yy in (-44, 44):
            cap = hw.cbore_hole(cap, 4, ((xa + xb) / 2, yy, L.z_el + 9), DOWN, 10, cbore_depth=Rsh)
            hw.screw("Head", 4, 20, ((xa + xb) / 2, yy, L.z_el + 9), DOWN)
        cap = cap.cut(G.cyl(40.2, 3, ((xb - 3) if sgn > 0 else xa, 0, L.z_el), X))
        _reg(reg, "El_Bearing_Cap_%s" % side_name(sgn), cap, "Head", "white", print_up=DOWN, notes="Rounded pillow-block cap, 2x M4x20 into nuts trapped in the shoulder; holds the upper bushing half.")
        for half, zsgn in (("Lower", -1), ("Upper", 1)):
            bx0 = (x0 + 2) if sgn > 0 else -(x0 + 42)
            bsh = G.tube(L.r_bush, L.r_axle + 0.25, 40, (bx0, 0, L.z_el), X)
            bsh = bsh.common(G.box(40, 80, 40, (bx0, -40, L.z_el if zsgn > 0 else L.z_el - 40)))
            _reg(reg, "El_Bushing_%s_%s" % (side_name(sgn), half), bsh, "Head", "white", print_up=(0, 0, zsgn), notes="Half of the split plain bushing (36/30.5 x 40). Grease.")

    for sgn in (1, -1):
        xso = sgn * xs
        cap = G.cyl(40, 7, (xso - sgn * 3 if sgn > 0 else xso - 4, 0, L.z_el), X)
        cap = cap.cut(G.tube(41, 38.5, 1.2, ((xso + 4 - 1.2) if sgn > 0 else (xso - 4), 0, L.z_el), X))
        for ang in (205, 245, 295, 335):
            py, pz = 33 * math.cos(math.radians(ang)), 33 * math.sin(math.radians(ang))
            seat = (xso + sgn * 4 - sgn * 3.0, py, L.z_el + pz)
            cap = hw.cbore_hole(cap, 3, seat, (-sgn, 0, 0), 3.0 + 4, cbore_depth=3.0); hw.screw("Head", 3, 12, seat, (-sgn, 0, 0))
        if sgn < 0:
            x_board = -(L.x_stub_end) - 2.0
            col = G.cyl(13, x_board - (xso - 4 + 2.2 + 0.01), (xso - 4 + 2.2 + 0.01, 0, L.z_el), PX)     # from behind the window seat to the board face
            col = col.cut(G.cbox(3.2, 20.5, 20.5, (x_board - 1.6, 0, L.z_el)))
            col = col.cut(G.box(40, 5, 4, (xso - 6, -2.5, L.z_el + 8)))
            for dy in (-7, 7):                                   # AS5600 board: 2x M3 into nuts slid in from the column side
                col = hw.clear_hole(col, 3, (x_board - 3.2, dy, L.z_el + 7), NX, 6)
                col = hw.nut_trap(col, 3, (x_board - 3.2 - 5.5, dy, L.z_el + 7), NX, (0, 1 if dy > 0 else -1, 0), 8, extra_len=2)
                hw.screw("Head", 3, 10, (x_board - 1.6, dy, L.z_el + 7), NX); hw.nut("Head", 3, (x_board - 3.2 - 5.5 - NUT[3][1] / 2, dy, L.z_el + 7), NX)
            cap = cap.cut(G.cyl(20, 2.2, (xso - 4 - 0.01, 0, L.z_el), X))
            cap = G.fuse(cap, col)
            _reg(reg, "El_Axis_Cap_L", cap, "Head", "grey", print_up=NX, notes="Left axis cap: AS5600 column (2 mm off the stub magnet), translucent window; 4x M3x12 into bore-loaded nuts.")
            _reg(reg, "Status_Window", G.cyl(19.7, 2, (xso - 4, 0, L.z_el), X), "Head", "clear", print_up=NX, notes="Translucent disc in the left cap.")
        else:
            _reg(reg, "El_Axis_Cap_R", cap, "Head", "grey", print_up=PX, notes="Right axis cap over the stub end; 4x M3x12 into bore-loaded nuts.")
    scov = G.cyl(46, 7, (xs - 3, 0, L.z_elpin), X)
    scov = scov.cut(G.tube(47, 44.5, 1.2, (xs + 4 - 1.2, 0, L.z_elpin), X))
    scov = G.fuse(scov, G.cbox(xs - 3 - (L.x_servo0 + ln) - 0.3, w - 6, h - 6, ((xs - 3 + L.x_servo0 + ln + 0.3) / 2, 0, L.z_elpin)))
    scov = scov.cut(G.cbox(10, 14, 6, ((xs - 3 + L.x_servo0 + ln) / 2, 0, L.z_elpin + h / 2 - 5)))
    for ang in (45, 135, 225, 315):
        py, pz = 40 * math.cos(math.radians(ang)), 40 * math.sin(math.radians(ang))
        seat = (xs + 4 - 3.0, py, L.z_elpin + pz)
        scov = hw.cbore_hole(scov, 3, seat, NX, 3.0 + 4, cbore_depth=3.0); hw.screw("Head", 3, 12, seat, NX)
    _reg(reg, "El_Servo_Cover", scov, "Head", "grey", print_up=PX, notes="Round cover over the servo pocket; integral spacer holds the servo. 4x M3x12 into pocket-loaded nuts.")
    el_servo = G.cbox(ln, w, h, (L.x_servo0 + ln / 2, 0, L.z_elpin))
    el_servo = G.fuse(el_servo, G.cyl(P["servo_shaft_d"] / 2, P["servo_shaft_len"], (L.x_servo0 - P["servo_shaft_len"], 0, L.z_elpin), X))
    _reg(reg, "El_Servo_placeholder", el_servo, "Head", "steel", printed=False, notes="Your servo (placeholder box) inside the right shoulder.")


# ============================================================== HEAD (puck, AZ housing, yoke plate)
def build_head(P, L, reg, hw):
    puck = G.fuse(G.cyl(L.R_lip, L.z_lip1 - L.z_puck0, (0, 0, L.z_puck0)), G.cyl(L.R_body, L.z_deck - L.z_lip1 + 0.01, (0, 0, L.z_lip1 - 0.01)))
    puck = puck.cut(G.tube(L.R_lip + 1, L.R_lip - 1.5, 1.0, (0, 0, L.z_puck0)))
    puck = puck.cut(G.hex_prism(22.4, 10.5, (0, 0, L.z_puck0 - 0.5)))
    puck = puck.cut(G.cyl(4.5, 40, (0, 0, L.z_puck0)))
    sx, sy = L.az_pinion_c
    puck = puck.cut(G.cbox(P["servo_w"] + 1, P["servo_h"] + 1, 40, (sx, sy, L.z_puck0 + 4 + 20)))
    puck = puck.cut(G.cyl(7, 10, (sx, sy, L.z_puck0 - 0.5)))
    # AZ servo clamp feet: M3 down through the puck, nuts in hex pockets on the puck underside
    for dx in (-1, 1):
        p = (sx + dx * (P["servo_w"] / 2 + 8), sy, L.z_deck)
        puck = hw.clear_hole(puck, 3, p, DOWN, 31); puck = hw.nut_pocket(puck, 3, (p[0], p[1], L.z_puck0), UP, depth=3.0)
        hw.nut("Head", 3, (p[0], p[1], L.z_puck0), UP)
    ix, iy = L.idler_c
    puck = hw.clear_hole(puck, 4, (ix, iy, L.z_puck0), UP, 30); puck = hw.nut_pocket(puck, 4, (ix, iy, L.z_deck), DOWN, depth=3.6)
    hw.screw("Head", 4, 45, (ix, iy, L.z_sens0 - 0.8), UP); hw.washer("Head", 4, (ix, iy, L.z_sens0 - 0.8), DOWN); hw.nut("Head", 4, (ix, iy, L.z_deck - 3.5), UP)
    gx, gy = L.sens_c
    puck = puck.cut(G.cyl(4.2, 6.5, (gx, gy, L.z_puck0 - 0.5)))
    puck = puck.cut(G.cbox(22, 34, 40, (gx, gy, L.z_puck0 + 7.5 + 20)))
    puck = puck.cut(G.box(8, 30, 40, (gx - 4, gy + 14, L.z_puck0 + 12)))
    for dy in (-13, 13):                                         # AS5600 board: M3x12 down into underside hex pockets
        p = (gx, gy + dy, L.z_puck0 + 7.5)
        puck = hw.clear_hole(puck, 3, p, DOWN, 8); puck = hw.nut_pocket(puck, 3, (p[0], p[1], L.z_puck0), UP, depth=3.0)
        hw.screw("Head", 3, 12, (gx, gy + dy, L.z_puck0 + 9.1), DOWN); hw.nut("Head", 3, (p[0], p[1], L.z_puck0), UP)
    for dxk in (-1, 1):                                          # keeper: M3x45 from the deck down through the puck, nut under the keeper
        p = (gx + dxk * 30, gy, L.z_deck)
        puck = hw.cbore_hole(puck, 3, (p[0], p[1], L.z_deck - 3.5), DOWN, 30, cbore_depth=3.5)
        hw.screw("Head", 3, 45, (p[0], p[1], L.z_deck - 3.5), DOWN)
    pillar_pts = G.pattern_circle(4, L.pillar_r, 0, 45)
    for p in pillar_pts:                                         # pillar feet: M4x30 from below, nuts trapped in the pillar feet
        puck = hw.cbore_hole(puck, 4, (p[0], p[1], L.z_puck0 + 8), UP, 22, cbore_depth=8); hw.screw("Head", 4, 30, (p[0], p[1], L.z_puck0 + 8), UP)
    puck = puck.cut(G.cyl(6, 40, (-30, 30, L.z_puck0 - 1))); puck = puck.cut(G.cyl(6, 40, (30, -30, L.z_puck0 - 1)))
    _reg(reg, "Head_Puck", puck, "Head", "white", notes="Rotating deck: 15 mm journal lip + thrust face (grease). Nuts for the deck hardware sit in hex pockets on its underside.")

    clamp = G.hex_prism(22, 10, (0, 0, L.z_puck0)).cut(G.cyl(P["slipring_stub_d"] / 2 + 0.1, 12, (0, 0, L.z_puck0 - 1)))
    clamp = clamp.cut(G.box(1.2, 14, 12, (-0.6, 0, L.z_puck0 - 1)))
    clamp = hw.clear_hole(clamp, 3, (-12, 7, L.z_puck0 + 5), PX, 24); clamp = hw.nut_pocket(clamp, 3, (8.5, 7, L.z_puck0 + 5), PX, depth=2.7)
    hw.screw("Head", 3, 16, (-11, 7, L.z_puck0 + 5), PX); hw.nut("Head", 3, (8.5, 7, L.z_puck0 + 5), PX)
    _reg(reg, "SlipRing_Stub_Clamp", clamp, "Head", "white", notes="Hex split collar keyed into the puck; M3x16 pinch bolt + nut.")

    w, h, ln, t = P["servo_w"], P["servo_h"], P["servo_len"], P["servo_wall"]
    ztop = L.z_puck0 + 4 + ln
    bar = G.cbox(w + 12, 12, 4, (sx, sy, ztop + 2)); legs = None
    for dx in (-1, 1):
        leg = G.cbox(5, 12, ztop + 4 - L.z_deck, (sx + dx * (w / 2 + 3.0), sy, (ztop + 4 + L.z_deck) / 2))
        foot = G.cbox(8, 12, 4, (sx + dx * (w / 2 + 8), sy, L.z_deck + 2))
        foot = hw.cbore_hole(foot, 3, (sx + dx * (w / 2 + 8), sy, L.z_deck + 4), DOWN, 4, cbore_depth=0.01); hw.screw("Head", 3, 35, (sx + dx * (w / 2 + 8), sy, L.z_deck + 4), DOWN)
        legs = G.fuse(leg, foot) if legs is None else G.fuse(legs, leg, foot)
    _reg(reg, "Az_Servo_Clamp", G.fuse(bar, legs), "Head", "white", notes="Saddle pinning the AZ servo; 2x M3x35 through the deck into underside nuts.")

    # ---- AZ housing: 232 mm drum aligned with the pedestal, bottom ring on the puck, pillars tied to the wall by ribs
    Rd, wall = L.R_drum, 4.0
    drum = G.tube(Rd, Rd - wall, L.z_drum1 - L.z_deck, (0, 0, L.z_deck))
    drum = G.fuse(drum, G.tube(Rd - wall + 0.01, L.pillar_r - 10, 4, (0, 0, L.z_deck)))                 # bottom ring on the puck
    drum = drum.cut(G.cbox(P["servo_w"] + 6, P["servo_h"] + 6, 6, (sx, sy, L.z_deck + 2)))                  # clear the AZ servo
    drum = drum.cut(G.cbox(26, 38, 6, (gx, gy, L.z_deck + 2)))                                              # clear the AS5600 well
    drum = drum.cut(G.tube(Rd + 1, Rd - 1.2, 1.4, (0, 0, L.z_drum1 - 10)))
    for p in pillar_pts:
        pil = G.cyl(9, L.z_drum1 - L.z_deck, (p[0], p[1], L.z_deck))
        ang = math.degrees(math.atan2(p[1], p[0]))
        rib = G.rot(G.box(Rd - wall - L.pillar_r + 2, 8, L.z_drum1 - L.z_deck, (L.pillar_r - 1, -4, L.z_deck)), Z, ang)
        pil = G.fuse(pil, rib)
        pil = hw.clear_hole(pil, 4, (p[0], p[1], L.z_deck), UP, 12)
        pil = hw.nut_trap(pil, 4, (p[0], p[1], L.z_deck + 9), UP, (-p[0] / L.pillar_r, -p[1] / L.pillar_r, 0), 10, extra_len=3)
        pil = hw.clear_hole(pil, 4, (p[0], p[1], L.z_drum1), DOWN, 12)
        pil = hw.nut_trap(pil, 4, (p[0], p[1], L.z_drum1 - 9), DOWN, (-p[0] / L.pillar_r, -p[1] / L.pillar_r, 0), 10, extra_len=3)
        hw.nut("Head", 4, (p[0], p[1], L.z_deck + 9 - NUT[4][1] / 2), UP); hw.nut("Head", 4, (p[0], p[1], L.z_drum1 - 9 - NUT[4][1] / 2), UP)
        drum = G.fuse(drum, pil)
    drum = drum.cut(G.cbox(14, 10, 8, (0, -Rd, L.z_deck + 22)))
    for i in range(4):
        drum = drum.cut(G.cbox(28, 10, 2, (0, Rd, L.z_deck + 40 + i * 10)))
    drum = G.stencil_text_cut(drum, P["logo_text"], P["logo_font"], 12, 20, at=(0, -Rd + 8, L.z_deck + 55), axis=NY)
    _reg(reg, "Az_Drum", drum, "Head", "white", notes="Rotating azimuth housing (same diameter as the pedestal) = electronics bay. 4 pillars + ribs carry the yoke; M4x30 from below, M4x16 from above, nuts trapped in the pillars.")
    pipe = G.annular_sector(Rd - wall - 2.6, Rd - wall - 0.6, 250, 290, 22, (0, 0, L.z_deck + 44), rounded=False)
    _reg(reg, "Logo_Light_Pipe", pipe, "Head", "clear", notes="Curved translucent plate behind the drum stencil (glue), LED behind.")

    px_, py_, ph = P["yoke_plate_x"], P["yoke_plate_y"], P["yoke_plate_h"]
    plate = G.cbox(px_, py_, ph, (0, 0, L.z_drum1 + ph / 2))
    plate = fillet_edges(plate, 34.0, lambda e: abs(e.Vertexes[0].Z - e.Vertexes[1].Z) > ph - 1)
    plate = G.fuse(plate, G.cyl(Rd + 2, ph, (0, 0, L.z_drum1)))                                   # covers the whole housing top
    plate = fillet_edges(plate, 4.0, lambda e: abs(e.Vertexes[0].Z - L.z_plate1) < 1e-3 and abs(e.Vertexes[1].Z - L.z_plate1) < 1e-3)
    plate = plate.cut(G.cyl(42, ph + 2, (0, 0, L.z_drum1 - 1)))
    for p in pillar_pts:
        plate = hw.cbore_hole(plate, 4, (p[0], p[1], L.z_plate1 - 4.5), DOWN, ph, cbore_depth=4.5); hw.screw("Head", 4, 16, (p[0], p[1], L.z_plate1 - 4.5), DOWN)
    for sgn in (-1, 1):
        for fx in (L.x_arm_in + 6, L.x_arm_out - 6):
            for fy in (-40, 40):
                plate = hw.cbore_hole(plate, 4, (sgn * fx, fy, L.z_drum1 + 4.5), UP, ph, cbore_depth=4.5); hw.screw("Head", 4, 25, (sgn * fx, fy, L.z_drum1 + 4.5), UP)
        plate = plate.cut(G.cyl(5, ph + 2, (sgn * (L.x_arm_in - 8), -L.arm_halfwidth(L.z_plate1) + 9, L.z_drum1 - 1)))
    for pt in G.pattern_circle(6, 50, 0, 0):                     # access cover: M3x25 down into hex pockets on the plate underside
        plate = hw.clear_hole(plate, 3, (pt[0], pt[1], L.z_plate1), DOWN, ph + 1); plate = hw.nut_pocket(plate, 3, (pt[0], pt[1], L.z_drum1), UP, depth=3.0)
        hw.nut("Head", 3, (pt[0], pt[1], L.z_drum1), UP)
    _reg(reg, "Yoke_Plate", plate, "Head", "white", notes="Broad yoke base / drum lid: pillars 4x M4x16 from above (nuts in the pillars), arm feet 4x M4x25 each from below (nuts in the arm feet), round access cover 6x M3x25 (nuts under the plate).")
    acc = G.cyl(58, 4, (0, 0, L.z_plate1))
    acc = fillet_edges(acc, 1.5, lambda e: e.Curve.TypeId == "Part::GeomCircle" and abs(e.Curve.Radius - 58) < 1e-3 and abs(e.Vertexes[0].Z - (L.z_plate1 + 4)) < 1e-3)
    acc = G.fuse(acc, G.cyl(41.5, 3, (0, 0, L.z_plate1 - 3)))
    for pt in G.pattern_circle(6, 50, 0, 0):
        acc = hw.cbore_hole(acc, 3, (pt[0], pt[1], L.z_plate1 + 4 - 3), DOWN, 3 + 1, cbore_depth=3); hw.screw("Head", 3, 25, (pt[0], pt[1], L.z_plate1 + 1), DOWN)
    _reg(reg, "Yoke_Access_Cover", acc, "Head", "grey", notes="Round access cover on the yoke plate; 6x M3x25 + nuts under the plate.")

    az_servo = G.cbox(w, h, ln, (sx, sy, L.z_puck0 + 4 + ln / 2))
    az_servo = G.fuse(az_servo, G.cyl(P["servo_shaft_d"] / 2, P["servo_shaft_len"], (sx, sy, L.z_puck0 + 4 - P["servo_shaft_len"])))
    _reg(reg, "Az_Servo_placeholder", az_servo, "Head", "steel", printed=False, notes="Your servo (placeholder box).")
    keeper = G.cbox(68, 10, 3, (gx, gy, L.z_sens0 - 2))
    keeper = G.fuse(keeper, G.cbox(4, 10, L.z_puck0 - (L.z_sens0 - 0.5), (gx - 32, gy, (L.z_puck0 + L.z_sens0 - 0.5) / 2)),
                    G.cbox(4, 10, L.z_puck0 - (L.z_sens0 - 0.5), (gx + 32, gy, (L.z_puck0 + L.z_sens0 - 0.5) / 2)))
    for dxk in (-1, 1):
        keeper = hw.clear_hole(keeper, 3, (gx + dxk * 30, gy, L.z_sens0 - 3.5), UP, 3)
        keeper = hw.nut_pocket(keeper, 3, (gx + dxk * 30, gy, L.z_sens0 - 3.5), UP, depth=2.7)
        hw.nut("Head", 3, (gx + dxk * 30, gy, L.z_sens0 - 3.5), UP)
    _reg(reg, "Sensor_Gear_Keeper", keeper, "Head", "white", print_up=DOWN, notes="Retains the AZ sensor gear on its pin; 2x M3x45 from the deck, nuts in the keeper.")


# ============================================================== GEARS
def build_gears(P, L, reg, hw):
    m, fw = L.m, L.fw
    px, py = L.az_pinion_c
    pin = G.spur_gear(P["az_pinion_teeth"], m, fw, hub_d=18, hub_h=L.z_pin_hub1 - L.z_ring1, backlash=P["gear_backlash"], at=(0, 0, L.z_ring0))
    pin = G.rot(pin, Z, 180.0 / P["az_pinion_teeth"]); pin.translate(V(px, py, 0))
    pin = pin.cut(G.cyl(P["servo_shaft_d"] / 2 + 0.15, 8, (px, py, L.z_pin_hub1 - 7)))
    pin = hw.cbore_hole(pin, 3, (px, py, L.z_pin_hub1 - 7), DOWN, 12, cbore_depth=7); hw.screw("AzPinion", 3, 10, (px, py, L.z_pin_hub1 - 7), DOWN)
    _reg(reg, "Az_Pinion", pin, "AzPinion", "orange", notes="%d T module %g; M3x10 axial screw into the servo's own shaft thread." % (P["az_pinion_teeth"], m))

    eg = G.spur_gear(P["el_gear_teeth"], m, 10, backlash=P["gear_backlash"], at=(L.x_gear0, 0, L.z_el), axis=X)
    eg = eg.cut(G.cyl(32, 12, (L.x_gear0 - 1, 0, L.z_el), X))
    for k in range(6):
        a0 = 60 * k + 12
        eg = eg.cut(G.annular_sector(40, 58, a0, a0 + 20, 12, (L.x_gear0 - 1, 0, L.z_el), X))
    for p in G.pattern_circle(4, 40, 0, 45):                       # 4x M4x20 through the gear and the hub wall, nuts inside the hub
        eg = hw.cbore_hole(eg, 4, (L.x_gear1 - 4.5, p[0], L.z_el + p[1]), NX, 4.5 + 1 + 1, cbore_depth=4.5)
        hw.screw("Cradle", 4, 20, (L.x_gear1 - 4.5, p[0], L.z_el + p[1]), NX)
    _reg(reg, "El_Gear", eg, "Cradle", "orange", print_up=PX, notes="%d T module %g bolted to the right hub wall (4x M4x20, nuts inside the hub)." % (P["el_gear_teeth"], m))

    ep = G.spur_gear(P["el_pinion_teeth"], m, 10, hub_d=18, hub_h=3, backlash=P["gear_backlash"], at=(L.x_gear0, 0, 0), axis=X)
    ep = G.rot(ep, X, 180.0 / P["el_pinion_teeth"]); ep.translate(V(0, 0, L.z_elpin))
    ep = ep.cut(G.cyl(P["servo_shaft_d"] / 2 + 0.15, 9, (L.x_gear1 + 3 - 8.5, 0, L.z_elpin), X))
    ep = hw.cbore_hole(ep, 3, (L.x_gear0 + 3, 0, L.z_elpin), PX, 12, cbore_depth=3); hw.screw("ElPinion", 3, 10, (L.x_gear0 + 3, 0, L.z_elpin), PX)
    _reg(reg, "El_Pinion", ep, "ElPinion", "orange", print_up=NX, notes="%d T module %g on the EL servo shaft; M3x10 axial into the servo's shaft thread." % (P["el_pinion_teeth"], m))

    sm = P["sensor_module"]
    ix, iy = L.idler_c
    idl = G.spur_gear(P["sensor_idler_teeth"], sm, L.z_sens1 - L.z_sens0, bore=4.5, backlash=P["gear_backlash"], at=(0, 0, L.z_sens0))
    idl = G.rot(idl, Z, 180.0 / P["sensor_idler_teeth"]); idl.translate(V(ix, iy, 0))
    _reg(reg, "Sensor_Idler", idl, "Idler", "orange", notes="Runs on the M4x45 axle.")
    gx, gy = L.sens_c
    sg = G.spur_gear(P["sensor_gear_teeth"], sm, L.z_sens1 - L.z_sens0, backlash=P["gear_backlash"], at=(gx, gy, L.z_sens0))
    sg = G.fuse(sg, G.cyl(4, 6, (gx, gy, L.z_sens1))); sg = sg.cut(G.cyl(3.1, 2.6, (gx, gy, L.z_sens1 + 6 - 2.5)))
    _reg(reg, "Sensor_Gear", sg, "Sensor", "orange", notes="1:1 with the sun; carries the AZ AS5600 magnet.")


# ============================================================== CRADLE (hollow hub with bolted lid; all nuts inside)
def build_cradle(P, L, reg, hw):
    hs, zc, bs, tw = L.hub, L.z_el, P["boom_size"], L.hub_wall
    hub = G.cbox(hs, hs, hs, (0, 0, zc))
    hub = hub.common(G.rot(G.cbox(hs * 1.22, hs * 1.22, hs * 1.6, (0, 0, zc)), Y, 45, (0, 0, zc)))
    hub = hub.common(G.rot(G.cbox(hs * 1.6, hs * 1.22, hs * 1.22, (0, 0, zc)), X, 45, (0, 0, zc)))
    hub = hub.common(G.rot(G.cbox(hs * 1.22, hs * 1.6, hs * 1.22, (0, 0, zc)), Z, 45, (0, 0, zc)))
    hub = hub.cut(G.cbox(hs - 2 * tw, hs - 2 * tw, hs, (0, 0, zc + tw)))                         # cavity, open at the top
    hub = hub.cut(G.cbox(hs - 2 * tw + 12, hs - 2 * tw + 12, 6, (0, 0, zc + hs / 2 - 3)))         # lid rebate
    for sgn in (1, -1):                                                                             # boom / cw flanges: M4x20 into nuts inside
        hub = hub.cut(G.cbox(bs + 0.4, 10.4, bs + 0.4, (0, sgn * (hs / 2 - 5), zc)))
        for (dx, dz) in ((-27, -27), (27, -27), (-27, 27), (27, 27)):
            p = (dx, sgn * hs / 2, zc + dz)
            hub = hw.clear_hole(hub, 4, p, (0, -sgn, 0), tw + 1); hw.nut("Cradle", 4, (dx, sgn * (hs / 2 - tw), zc + dz), (0, -sgn, 0))
    for sgn in (1, -1):                                                                             # stub flanges: spigot recess + M4x20 into nuts inside
        hub = hub.cut(G.cyl(15.3, 6.3, (sgn * (hs / 2 + 0.01), 0, zc), (-sgn, 0, 0)))
        for p in G.pattern_circle(4, 20, 0, 45):
            q = (sgn * hs / 2, p[0], zc + p[1])
            hub = hw.clear_hole(hub, 4, q, (-sgn, 0, 0), tw + 1); hw.nut("Cradle", 4, (sgn * (hs / 2 - tw), p[0], zc + p[1]), (-sgn, 0, 0))
    for p in G.pattern_circle(4, 40, 0, 45):                                                       # EL gear: M4x20 into nuts inside
        q = (hs / 2, p[0], zc + p[1])
        hub = hw.clear_hole(hub, 4, q, NX, tw + 1); hw.nut("Cradle", 4, (hs / 2 - tw, p[0], zc + p[1]), NX)
    hub = hub.cut(G.cyl(5, tw + 2, (0, 0, zc - hs / 2 - 1)))                                        # coax through the floor
    # lid bolts: 4x M3 down through the lid into nuts side-loaded from inside the cavity walls
    for (dx, dy) in ((-32, -32), (32, -32), (-32, 32), (32, 32)):
        hub = hw.clear_hole(hub, 3, (dx, dy, zc + hs / 2), DOWN, 12)
        hub = hw.nut_trap(hub, 3, (dx, dy, zc + hs / 2 - 9), DOWN, (-dx / 32, 0, 0) if abs(dx) > 0 else (0, -1, 0), 8, extra_len=3)
        hw.nut("Cradle", 3, (dx, dy, zc + hs / 2 - 9 - NUT[3][1] / 2), UP)
    _reg(reg, "Cradle_Hub", hub, "Cradle", "black", notes="Hollow faceted hub, open top with a bolted lid: every flange bolt (stubs, gear, boom, counterweight arm) ends in a nut inside the cavity.")
    lid = G.cbox(hs - 2 * tw + 11.4, hs - 2 * tw + 11.4, 6, (0, 0, zc + hs / 2 - 3))
    lid = G.fuse(lid, G.cbox(hs - 8, hs - 8, 2, (0, 0, zc + hs / 2 + 1)))
    lid = lid.common(G.rot(G.cbox(hs * 1.22, hs * 1.22, hs * 1.6, (0, 0, zc)), Y, 45, (0, 0, zc)).common(G.rot(G.cbox(hs * 1.22, hs * 1.6, hs * 1.22, (0, 0, zc)), Z, 45, (0, 0, zc))))
    lid = lid.cut(G.cyl(5, 12, (0, 0, zc + hs / 2 - 5)))
    for (dx, dy) in ((-32, -32), (32, -32), (-32, 32), (32, 32)):
        lid = hw.cbore_hole(lid, 3, (dx, dy, zc + hs / 2 + 2 - 3), DOWN, 8, cbore_depth=3.0); hw.screw("Cradle", 3, 20, (dx, dy, zc + hs / 2 - 1), DOWN)
    _reg(reg, "Cradle_Lid", lid, "Cradle", "black", print_up=DOWN, notes="Hub lid, 4x M3x20 into nuts trapped in the hub walls. Fit after all flange nuts are in.")

    for sgn in (1, -1):
        xf = sgn * hs / 2
        x_end = sgn * L.x_stub_end
        stub = G.cyl(L.r_axle, abs(x_end - xf), (min(xf, x_end), 0, zc), X)
        fl = G.cyl(30, 6, (xf, 0, zc), (sgn, 0, 0))
        stub = G.fuse(stub, fl, G.cyl(15, 6, (xf, 0, zc), (-sgn, 0, 0)))
        for p in G.pattern_circle(4, 20, 0, 45):
            seat = (xf + sgn * 6 - sgn * 4.5, p[0], zc + p[1])
            stub = hw.cbore_hole(stub, 4, seat, (-sgn, 0, 0), 4.5 + 1 + 6, cbore_depth=4.5); hw.screw("Cradle", 4, 20, seat, (-sgn, 0, 0))
        if sgn < 0:
            stub = stub.cut(G.cyl(3.1, 3, (x_end + 3, 0, zc), NX))
        _reg(reg, "El_Stub_%s" % side_name(sgn), stub, "Cradle", "black", print_up=(-sgn, 0, 0),
             notes="EL stub axle, flange 4x M4x20 through the hub wall into nuts inside; turns in the split bushings. %s" % ("" if sgn > 0 else "Magnet in the end face."))

    def square_arm(y_root_sign, y_end, name, color, notes, extra=None):
        yr = y_root_sign * hs / 2; length = abs(y_end - yr); yc_ = (yr + y_end) / 2
        tube_ = G.cbox(bs, length, bs, (0, yc_, zc)).cut(G.cbox(bs - 8, length + 2, bs - 8, (0, yc_, zc)))
        tube_ = G.fuse(tube_, G.cbox(bs, 12, bs, (0, yr + y_root_sign * 6, zc)), G.cbox(bs - 0.4, 10, bs - 0.4, (0, yr - y_root_sign * 5, zc)))
        fl = G.cbox(64, 6, 64, (0, yr + y_root_sign * 3, zc)).common(G.rot(G.cbox(80, 8, 80, (0, yr + y_root_sign * 3, zc)), Y, 45, (0, 0, zc)))
        tube_ = G.fuse(tube_, fl)
        for (dx, dz) in ((-27, -27), (27, -27), (-27, 27), (27, 27)):
            seat = (dx, yr + y_root_sign * 1.5, zc + dz)
            tube_ = hw.cbore_hole(tube_, 4, seat, (0, -y_root_sign, 0), 2, cbore_depth=4.5); hw.screw("Cradle", 4, 20, seat, (0, -y_root_sign, 0))
        if extra: tube_ = extra(tube_)
        _reg(reg, name, tube_, "Cradle", color, notes=notes)

    def boom_end(tb):
        fl = G.fuse(G.cyl(32, 8, (0, L.y_boom_end - 8, zc), Y), G.cbox(bs, 4, bs, (0, L.y_boom_end - 10, zc)))
        for p in G.pattern_circle(4, 24, 0, 45):
            fl = hw.clear_hole(fl, 5, (p[0], L.y_boom_end - 8, zc + p[1]), PY, 8)
        return G.fuse(tb, fl).cut(G.cyl(6, 10, (0, L.y_boom_end - 9, zc), Y))
    square_arm(1, L.y_boom_end - 8, "Dish_Boom", "black", "Square boom to the dish offset; root flange 4x M4x20 into nuts inside the hub; round end flange takes either adapter (4x M5 + nuts).", boom_end)

    def cw_holes(tb):
        for i in range(6):
            tb = hw.clear_hole(tb, 4, (-bs / 2 - 1, L.y_cw_end + 20 + i * 22, zc), PX, bs + 2)
        return tb
    square_arm(-1, L.y_cw_end, "Counterweight_Arm", "black", "Square tube with a row of M4 holes for the canister collar; root flange 4x M4x20 into nuts inside the hub.", cw_holes)

    ad = P["adapter_flange_d"]
    plate = G.cyl(ad / 2, 8, (0, L.y_boom_end, zc), Y)
    for ang in range(0, 360, 45):
        plate = plate.cut(G.rot(G.cbox(7, 10, 24, (0, L.y_boom_end + 4, zc + 46)), Y, ang, (0, 0, zc)))
    for ang in range(0, 360, 45):
        w_ = G.rot(G.cbox(4, 10, 40, (0, L.y_boom_end + 4, zc + 28)), Y, ang + 22.5, (0, 0, zc))
        plate = plate.cut(w_.common(G.tube(ad / 2 - 6, 30, 12, (0, L.y_boom_end - 1, zc), Y)))
    plate = plate.cut(G.cyl(6, 10, (0, L.y_boom_end - 1, zc), Y))
    for p in G.pattern_circle(4, 24, 0, 45):
        plate = hw.cbore_hole(plate, 5, (p[0], L.y_boom_end + 8 - 5.5, zc + p[1]), NY, 16, cbore_depth=5.5)
        hw.screw("Cradle", 5, 20, (p[0], L.y_boom_end + 8 - 5.5, zc + p[1]), NY); hw.nut("Cradle", 5, (p[0], L.y_boom_end - 8, zc + p[1]), NY)
    _reg(reg, "Dish_Adapter_Plate", plate, "Cradle", "grey", print_up=PY, notes="Adapter A: slotted plate; 4x M5x20 + nuts.")
    stub = G.cyl(32, 8, (0, L.y_boom_end, zc), Y)
    for p in G.pattern_circle(4, 24, 0, 45):
        stub = hw.cbore_hole(stub, 5, (p[0], L.y_boom_end + 8 - 5.5, zc + p[1]), NY, 16, cbore_depth=5.5)
    stub = G.fuse(stub, G.tube(24.15, 20, 100, (0, L.y_boom_end + 8, zc), Y))
    _reg(reg, "Dish_Adapter_PipeStub", stub, "Cradle", "grey", print_up=PY, hidden=True, notes="Adapter B: 48.3 mm pipe stub.")

    cd, cl = P["cw_canister_d"], P["cw_canister_len"]; yc0 = L.y_cw_end
    can = G.cyl(cd / 2, cl, (0, yc0, zc), NY).cut(G.cyl(cd / 2 - 2.5, cl - 4, (0, yc0 - 4, zc), NY))
    collar = G.cbox(bs + 8, 60, bs + 8, (0, yc0 + 26, zc)).cut(G.cbox(bs + 0.5, 62, bs + 0.5, (0, yc0 + 26, zc)))
    can = G.fuse(can, collar)
    for yy in (yc0 + 20, yc0 + 42):
        can = hw.clear_hole(can, 4, (-bs / 2 - 5, yy, zc), PX, bs + 10); hw.screw("Cradle", 4, 55, (-bs / 2 - 4, yy, zc), PX); hw.nut("Cradle", 4, (bs / 2 + 4, yy, zc), PX)
    for i in range(8):
        can = G.fuse(can, G.rot(G.cbox(3, cl - 30, 2, (0, yc0 - cl / 2, zc + cd / 2)), Y, 45 * i + 22.5, (0, 0, zc)))
    for ang in (0, 180):                                           # cap bolts: M3x10 through the cap skirt and canister wall, nuts inside
        can = hw.clear_hole(can, 3, ((cd / 2 + 2) * math.cos(math.radians(ang)), yc0 - cl + 5, zc + (cd / 2 + 2) * math.sin(math.radians(ang))), (-math.cos(math.radians(ang)), 0, -math.sin(math.radians(ang))), 6)
        hw.nut("Cradle", 3, ((cd / 2 - 2.5) * math.cos(math.radians(ang)), yc0 - cl + 5, zc + (cd / 2 - 2.5) * math.sin(math.radians(ang))), (-math.cos(math.radians(ang)), 0, -math.sin(math.radians(ang))))
    _reg(reg, "Counterweight_Canister", can, "Cradle", "clear", print_up=PY, notes="Translucent canister (~1.3 L); collar 2x M4x55 + nuts; cap 2x M3x10 with nuts inside the rim.")
    cap = G.cyl(cd / 2 + 2, 10, (0, yc0 - cl + 1, zc), NY).cut(G.cyl(cd / 2 + 0.25, 8, (0, yc0 - cl + 1, zc), NY))
    cap = G.fuse(cap, G.cyl(cd / 2 - 2.5 - 0.3, 11.5, (0, yc0 - cl + 4, zc), NY))
    cap = cap.cut(G.text_solid(P["logo_text"], P["logo_font"], 12, 1.0, at=(0, yc0 - cl - 8, zc), axis=NY))
    for ang in (0, 180):
        p = ((cd / 2 + 2) * math.cos(math.radians(ang)), yc0 - cl + 5, zc + (cd / 2 + 2) * math.sin(math.radians(ang)))
        d = (-math.cos(math.radians(ang)), 0, -math.sin(math.radians(ang)))
        cap = hw.clear_hole(cap, 3, p, d, 3); hw.screw("Cradle", 3, 10, p, d)
    _reg(reg, "Counterweight_Cap", cap, "Cradle", "orange", print_up=PY, notes="Press cap, engraved PERIGEE; 2x M3x10 + nuts.")

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
    px, py = L.az_pinion_c
    mk("_JM_AzPin_Head", (px, py, L.z_ring0 + 4), Z, "Head"); mk("_JM_AzPin_Pin", (px, py, L.z_ring0 + 4), Z, "AzPinion")
    mk("_JM_ElPin_Head", (L.x_gear0 + 3, 0, L.z_elpin), X, "Head"); mk("_JM_ElPin_Pin", (L.x_gear0 + 3, 0, L.z_elpin), X, "ElPinion")
    ix, iy = L.idler_c; gx, gy = L.sens_c
    mk("_JM_Idler_Head", (ix, iy, L.z_sens0 + 3), Z, "Head"); mk("_JM_Idler_Idler", (ix, iy, L.z_sens0 + 3), Z, "Idler")
    mk("_JM_Sens_Head", (gx, gy, L.z_sens0 + 3), Z, "Head"); mk("_JM_Sens_Sens", (gx, gy, L.z_sens0 + 3), Z, "Sensor")
    mk("_JM_Ring_Base", (0, 0, L.z_ring0 + 4), Z, "Base"); mk("_JM_Sun_Base", (0, 0, L.z_sens0 + 3), Z, "Base")
    mk("_JM_ElGear_Cradle", (L.x_gear0 + 3, 0, zc), X, "Cradle")


def build_all(P):
    L = Layout(P); reg = {}; hw = Hardware()
    build_base(P, L, reg, hw); build_arms(P, L, reg, hw); build_head(P, L, reg, hw)
    build_gears(P, L, reg, hw); build_cradle(P, L, reg, hw); build_markers(P, L, reg)
    return L, reg, hw
