"""Metric fastener library: ISO 4762 socket head cap screws, ISO 4032 hex nuts, heat-set inserts.
Every fastener is registered as an instance (kind + placement) so the assembly, viewer and BOM see the same hardware."""
import math
import FreeCAD as App
import Part
from .geom import V, Z, cyl, hex_prism, fuse

# d: (head_dia, head_height)  ISO 4762
SCREW = {2.5: (4.5, 2.5), 3: (5.5, 3.0), 4: (7.0, 4.0), 5: (8.5, 5.0), 6: (10.0, 6.0)}
# d: (across_flats, height)   ISO 4032
NUT = {3: (5.5, 2.4), 4: (7.0, 3.2), 5: (8.0, 4.7), 6: (10.0, 5.2)}
# d: (outer_dia, length, hole_dia_in_plastic)  common brass heat-set inserts
INSERT = {3: (4.6, 5.7, 4.0), 4: (5.6, 8.0, 5.6), 5: (6.4, 9.5, 6.4)}
# standard ISO 4762 lengths to snap to
LENGTHS = [6, 8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80]
CLEAR = {2.5: 2.9, 3: 3.4, 4: 4.5, 5: 5.5, 6: 6.6}   # clearance hole (medium fit)


def std_len(L):
    for s in LENGTHS:
        if s >= L - 0.01:
            return s
    return LENGTHS[-1]


def _rot_to(direction):
    """Rotation taking local +Z onto `direction`, robust for the antiparallel case (App.Rotation(Z, -Z) is ill-defined)."""
    d = V(*direction)
    d.normalize()
    if (d + Z).Length < 1e-9:
        return App.Rotation(V(1, 0, 0), 180)
    return App.Rotation(Z, d)


class Hardware:
    def __init__(self):
        self.kinds = {}      # key -> dict(shape, color, label)
        self.instances = []  # dict(kind, group, placement)

    # ---- kind shapes (local frame: axis +Z)
    def _kind(self, key, builder, color, label):
        if key not in self.kinds:
            self.kinds[key] = dict(shape=builder(), color=color, label=label)
        return key

    def screw(self, group, d, L, at, direction):
        """Socket head cap screw. `at` = head seating point, `direction` = shank direction (into the material)."""
        L = std_len(L)
        hd, hh = SCREW[d]
        key = "Screw_M%gx%d" % (d, L)

        def build():
            head = cyl(hd / 2, hh, (0, 0, 0))
            head = head.cut(hex_prism(0.5 * d + 0.5 if d < 4 else (3 if d == 4 else 4 if d == 5 else 5), hh * 0.6, (0, 0, hh * 0.4)))
            shank = cyl(d / 2 * 0.98, L, (0, 0, -L))
            return fuse(head, shank)
        self._kind(key, build, "steel", "M%g x %d socket head cap screw" % (d, L))
        # local: head z 0..hh (+Z), shank down -Z.  world: -Z -> direction  =>  +Z -> -direction
        plc = App.Placement(V(*at), _rot_to((-direction[0], -direction[1], -direction[2])))
        self.instances.append(dict(kind=key, group=group, placement=plc))
        return L

    def nut(self, group, d, at, direction):
        """Hex nut. `at` = bearing face position, `direction` = axis pointing away from the joint (nut extends along it)."""
        af, h = NUT[d]
        key = "Nut_M%g" % d

        def build():
            return hex_prism(af, h, (0, 0, 0)).cut(cyl(d / 2 * 0.98, h + 2, (0, 0, -1)))
        self._kind(key, build, "steel", "M%g hex nut" % d)
        self.instances.append(dict(kind=key, group=group, placement=App.Placement(V(*at), _rot_to(direction))))

    def insert(self, group, d, at, direction):
        """Heat-set insert. `at` = surface point, `direction` = into the material."""
        od, ln, hole = INSERT[d]
        key = "Insert_M%g" % d

        def build():
            body = cyl(od / 2, ln, (0, 0, -ln))
            body = body.cut(cyl(d / 2 * 0.98, ln + 2, (0, 0, -ln - 1)))
            for k in range(3):  # knurl bands
                body = body.cut(cyl(od / 2 + 1, 0.6, (0, 0, -ln + 1.2 + k * 1.8)).cut(cyl(od / 2 - 0.35, 0.6, (0, 0, -ln + 1.2 + k * 1.8))))
            return body
        self._kind(key, build, "brass", "M%g heat-set insert (%.1f mm hole)" % (d, hole))
        # local body along -Z from the surface; world -Z -> direction => +Z -> -direction
        self.instances.append(dict(kind=key, group=group, placement=App.Placement(V(*at), _rot_to((-direction[0], -direction[1], -direction[2])))))

    def washer(self, group, d, at, direction):
        key = "Washer_M%g" % d
        od = {3: 7, 4: 9, 5: 10, 6: 12}[d]

        def build():
            return cyl(od / 2, 0.8).cut(cyl(d / 2 + 0.2, 2, (0, 0, -0.5)))
        self._kind(key, build, "steel", "M%g washer" % d)
        self.instances.append(dict(kind=key, group=group, placement=App.Placement(V(*at), _rot_to(direction))))

    # ---- helpers that cut the matching features into a solid
    @staticmethod
    def cbore_hole(shape, d, at, direction, through, cbore_depth=None):
        """Clearance hole of length `through` starting at `at` along `direction`, plus a counterbore for the head at `at`
        going *against* the direction (i.e. the head sits in the recess above `at`)."""
        hd, hh = SCREW[d]
        dv = V(*direction); dv.normalize()
        a = V(*at)
        shape = shape.cut(cyl(CLEAR[d] / 2, through + 0.02, a - dv * 0.01, dv))
        depth = cbore_depth if cbore_depth is not None else hh + 1.0
        shape = shape.cut(cyl(hd / 2 + 0.6, depth + 0.01, a - dv * depth, dv))
        return shape

    @staticmethod
    def clear_hole(shape, d, at, direction, through):
        dv = V(*direction); dv.normalize()
        return shape.cut(cyl(CLEAR[d] / 2, through + 0.02, V(*at) - dv * 0.01, dv))

    @staticmethod
    def insert_hole(shape, d, at, direction, extra=1.5):
        od, ln, hole = INSERT[d]
        dv = V(*direction); dv.normalize()
        return shape.cut(cyl(hole / 2, ln + extra, V(*at) - dv * 0.01, dv))

    @staticmethod
    def nut_trap(shape, d, at, axis, open_dir, depth, extra_len=None):
        """Side-loaded captive nut slot: the nut sits at `at` with its axis along `axis`; the slot runs from there
        toward the open face along `open_dir` for `depth` mm so the nut slides in sideways. Also cuts the bolt
        clearance continuing past the nut by extra_len (default: nut height + 6)."""
        af, h = NUT[d]
        ax = V(*axis); ax.normalize(); od = V(*open_dir); od.normalize()
        a = V(*at)
        # slot cross-section: (af+0.4) across flats ... build a box aligned with (od, ax x od, ax)
        side = ax.cross(od); side.normalize()
        w, t = af + 0.5, h + 0.5
        # box centred on the nut, extended along od by depth
        L_ = depth + w / 2
        corner = a - side * (w / 2) - ax * (t / 2) - od * (w / 2)
        box = Part.makeBox(L_, w, t)
        m = App.Matrix()
        m.A11, m.A21, m.A31 = od.x, od.y, od.z
        m.A12, m.A22, m.A32 = side.x, side.y, side.z
        m.A13, m.A23, m.A33 = ax.x, ax.y, ax.z
        m.A14, m.A24, m.A34 = corner.x, corner.y, corner.z
        box.Placement = App.Placement(m)
        shape = shape.cut(box)
        ext = extra_len if extra_len is not None else h + 6
        shape = shape.cut(cyl(CLEAR[d] / 2, ext, a - ax * 0.01, ax))
        return shape

    @staticmethod
    def nut_pocket(shape, d, at, direction, depth=None):
        af, h = NUT[d]
        dv = V(*direction); dv.normalize()
        return shape.cut(hex_prism(af + 0.4, (depth or h) + 0.3, tuple(V(*at) - dv * 0.01), dv))

    def summary(self):
        from collections import Counter
        c = Counter(i["kind"] for i in self.instances)
        return sorted(((self.kinds[k]["label"], n) for k, n in c.items()), key=lambda t: t[0])
