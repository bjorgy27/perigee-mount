"""Geometry helpers on top of FreeCAD Part (pure OCC solids, no GUI)."""
import math
import FreeCAD as App
import Part

V = App.Vector
Z = V(0, 0, 1)
X = V(1, 0, 0)
Y = V(0, 1, 0)


def _vec(a):
    return a if isinstance(a, App.Base.Vector) else V(*a)


def cyl(r, h, at=(0, 0, 0), axis=Z):
    return Part.makeCylinder(r, h, V(*at), _vec(axis))


def tube(ro, ri, h, at=(0, 0, 0), axis=Z):
    return cyl(ro, h, at, axis).cut(cyl(ri, h, at, axis))


def box(lx, ly, lz, at=(0, 0, 0)):
    """Box with its corner at `at`."""
    return Part.makeBox(lx, ly, lz, V(*at))


def cbox(lx, ly, lz, center=(0, 0, 0)):
    """Box centred on `center`."""
    cx, cy, cz = center
    return Part.makeBox(lx, ly, lz, V(cx - lx / 2, cy - ly / 2, cz - lz / 2))


def sphere(r, at=(0, 0, 0)):
    return Part.makeSphere(r, V(*at))


def torus(R, r, at=(0, 0, 0), axis=Z):
    return Part.makeTorus(R, r, V(*at), _vec(axis))


def rot(shape, axis, deg, center=(0, 0, 0)):
    s = shape.copy()
    s.rotate(V(*center), V(*axis), deg)
    return s


def mv(shape, dx=0, dy=0, dz=0):
    s = shape.copy()
    s.translate(V(dx, dy, dz))
    return s


def fuse(*shapes):
    shapes = [s for s in shapes if s is not None]
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    return out.removeSplitter()


def cut(base, *tools):
    out = base
    for t in tools:
        if t is not None:
            out = out.cut(t)
    return out


def polygon_prism(pts2d, h, z0=0.0):
    """Extrude a closed 2D polygon (list of (x,y)) along +Z."""
    pts = [V(x, y, z0) for x, y in pts2d] + [V(pts2d[0][0], pts2d[0][1], z0)]
    wire = Part.makePolygon(pts)
    return Part.Face(wire).extrude(V(0, 0, h))


def hex_prism(af, h, at=(0, 0, 0), axis=Z):
    """Hexagonal prism, `af` across flats, axis along `axis` starting at `at`."""
    r = af / math.sqrt(3)  # circumradius
    pts = [(r * math.cos(math.radians(60 * i + 30)), r * math.sin(math.radians(60 * i + 30))) for i in range(6)]
    p = polygon_prism(pts, h)
    if _vec(axis) != Z:
        p = _align_z_to(p, axis)
    p.translate(V(*at))
    return p


def _align_z_to(shape, axis):
    axis = V(*axis) if not isinstance(axis, App.Base.Vector) else axis
    r = App.Rotation(Z, axis)
    s = shape.copy()
    s.Placement = App.Placement(V(0, 0, 0), r).multiply(s.Placement)
    return s


def stadium(lx, ly, h, at=(0, 0, 0)):
    """Rounded-end slot shape ("stadium") of overall size lx x ly, long axis Y."""
    r = lx / 2
    straight = ly - 2 * r
    s = cbox(lx, max(straight, 0.001), h, (0, 0, h / 2))
    s = fuse(s, cyl(r, h, (0, straight / 2, 0)), cyl(r, h, (0, -straight / 2, 0)))
    s.translate(V(*at))
    return s


# ---------------------------------------------------------------- gears

def involute_gear_face(teeth, module, external=True, pressure_angle=20.0, backlash=0.0):
    """Return a Part.Face of an involute gear outline centred at origin (uses FreeCAD's fcgear)."""
    from fcgear import fcgear  # ships with FreeCAD PartDesign
    doc = App.ActiveDocument
    tmp = doc.addObject("Part::Feature", "_tmpgear")
    try:
        import InvoluteGearFeature
        g = InvoluteGearFeature.makeInvoluteGear("_tmpInvolute")
        g.NumberOfTeeth = int(teeth)
        g.Modules = f"{module} mm"
        g.PressureAngle = f"{pressure_angle} deg"
        g.ExternalGear = bool(external)
        g.HighPrecision = False
        if backlash and external:
            # negative profile shift thins the tooth: ds = 2*x*m*tan(alpha); half the backlash per gear
            g.ProfileShiftCoefficient = -(backlash / 2) / (2 * module * math.tan(math.radians(pressure_angle)))
        doc.recompute()
        wire = Part.Wire(g.Shape.Edges)
        face = Part.Face(wire)
        doc.removeObject(g.Name)
    finally:
        doc.removeObject(tmp.Name)
    return face


def spur_gear(teeth, module, width, bore=0.0, hub_d=0.0, hub_h=0.0, backlash=0.0, at=(0, 0, 0), axis=Z):
    """External spur gear solid, axis along `axis`, starting at `at`."""
    face = involute_gear_face(teeth, module, True, backlash=backlash)
    g = face.extrude(V(0, 0, width))
    if hub_d and hub_h:
        g = fuse(g, cyl(hub_d / 2, hub_h, (0, 0, width)))
    if bore:
        g = g.cut(cyl(bore / 2, width + hub_h + 2, (0, 0, -1)))
    if _vec(axis) != Z:
        g = _align_z_to(g, axis)
    g.translate(V(*at))
    return g


def internal_ring_gear(teeth, module, width, outer_r, backlash=0.0, at=(0, 0, 0)):
    """Internal gear: annulus of radius outer_r with internal involute teeth."""
    face = involute_gear_face(teeth, module, False, backlash=backlash)
    teeth_solid = face.extrude(V(0, 0, width))  # fcgear internal outline = material of the ring
    ring = cyl(outer_r, width)
    # fcgear's internal profile is the tooth-space outline; keep material between outer_r and profile
    g = ring.cut(teeth_solid)  # fcgear's internal outline is the bore, so material = ring minus bore
    g.translate(V(*at))
    return g


def pitch_r(teeth, module):
    return teeth * module / 2.0


# ---------------------------------------------------------------- fasteners

def bolt_hole(d, length, at, axis=Z):
    return cyl(d / 2, length, at, axis)


def nut_pocket(af, h, at, axis=Z):
    return hex_prism(af, h, at, axis)


def countersink(d, at, axis=Z, depth=2.5):
    return Part.makeCone(d / 2 + depth, d / 2, depth, V(*at), _vec(axis))


def pattern_circle(n, r, z=0.0, start_deg=0.0):
    return [(r * math.cos(math.radians(start_deg + 360.0 * i / n)),
             r * math.sin(math.radians(start_deg + 360.0 * i / n)), z) for i in range(n)]


# ---------------------------------------------------------------- bearings

def v_groove_ring(pcd_r, ball_d, z, depth_frac=0.42):
    """Solid of revolution used to cut a 90-degree V groove for balls. Returns a solid ring
    that removes material where the ball sits (ball_d*depth_frac deep) - cut this from BOTH races."""
    # model as a torus slightly smaller than the ball so the ball touches at 4 points
    return torus(pcd_r, ball_d / 2 * 0.98, (0, 0, z))


def ball_ring(pcd_r, ball_d, z, n=None, axis=Z, center=(0, 0, 0)):
    """Compound of spheres around a circle (visual placeholder for the BBs)."""
    if n is None:
        n = int(math.floor(2 * math.pi * pcd_r / (ball_d * 1.02)))
    balls = []
    for i in range(n):
        a = 2 * math.pi * i / n
        balls.append(sphere(ball_d / 2, (pcd_r * math.cos(a), pcd_r * math.sin(a), 0)))
    c = Part.makeCompound(balls)
    if _vec(axis) != Z:
        c = _align_z_to(c, axis)
    c.translate(V(center[0], center[1], center[2] + (z if axis == Z else 0)))
    return c, n


# ---------------------------------------------------------------- text

def text_solid(text, font, size, depth, at=(0, 0, 0), axis=Z, rotate_deg=0.0):
    """Extruded text (for cut-outs). Lies in XY, extruded along Z, then aligned to axis."""
    import Draft
    doc = App.ActiveDocument
    ss = Draft.make_shapestring(text, font, size)
    doc.recompute()
    shp = ss.Shape.copy()
    doc.removeObject(ss.Name)
    bb = shp.BoundBox
    shp.translate(V(-bb.Center.x, -bb.Center.y, 0))
    solid = shp.extrude(V(0, 0, depth))
    if rotate_deg:
        solid.rotate(V(0, 0, 0), Z, rotate_deg)
    if _vec(axis) != Z:
        solid = _align_z_to(solid, axis)
    solid.translate(V(*at))
    return solid


def hex_grid_cutter(width, height, af, rib, depth, at=(0, 0, 0), axis=Z, keepout=None):
    """Honeycomb of hex prisms filling a width x height rectangle (centred), extruded `depth` along axis."""
    r = af / math.sqrt(3)
    pitch_x = af + rib
    pitch_y = (af + rib) * math.sqrt(3) / 2
    cutters = []
    ny = int(height // pitch_y) + 2
    nx = int(width // pitch_x) + 2
    for j in range(-ny, ny + 1):
        y = j * pitch_y
        for i in range(-nx, nx + 1):
            x = i * pitch_x + (pitch_x / 2 if j % 2 else 0)
            if abs(x) + r > width / 2 or abs(y) + r > height / 2:
                continue
            if keepout and keepout(x, y):
                continue
            cutters.append(hex_prism(af, depth, (x, y, 0)))
    if not cutters:
        return None
    c = Part.makeCompound(cutters)
    if _vec(axis) != Z:
        c = _align_z_to(c, axis)
    c.translate(V(*at))
    return c


def stencil_text_cut(target, text, font, size, depth, at, axis, rotate_deg=0.0, mirror_plane=None, bridge_frac=(0.22, -0.22), bridge_w=1.6):
    """Cut `text` through `target` and put thin stencil bridges back so letter counters (P, R, O...) stay attached.
    Bridges run along the text's baseline direction at fractions of `size` above/below the centre line."""
    txt = text_solid(text, font, size, depth, at=at, axis=axis, rotate_deg=rotate_deg)
    if mirror_plane is not None:
        txt = txt.mirror(V(*mirror_plane[0]), V(*mirror_plane[1]))
    bb = txt.BoundBox
    cutter = txt
    # bridges: thin slabs spanning the text bbox, thickness along the letters' vertical direction
    ax = V(*axis)
    for f in bridge_frac:
        # local text frame: extrusion along `axis`; letters' up direction = rotated local Y
        up = App.Rotation(Z, ax).multVec(App.Rotation(Z, rotate_deg).multVec(V(0, 1, 0)))
        if mirror_plane is not None:
            n = V(*mirror_plane[1]); up = up - n * (2 * up.dot(n))
        along = App.Rotation(Z, ax).multVec(App.Rotation(Z, rotate_deg).multVec(V(1, 0, 0)))
        c = V(bb.Center.x, bb.Center.y, bb.Center.z) + up * (f * size)
        L = max(bb.XLength, bb.YLength, bb.ZLength) + 4
        slab = Part.makeBox(L, bridge_w, depth + 4, V(-L / 2, -bridge_w / 2, -(depth + 4) / 2))
        r = App.Rotation(V(1, 0, 0), along)  # x -> along
        slab.Placement = App.Placement(V(0, 0, 0), r).multiply(slab.Placement)
        # make slab's local y match `up`
        y_now = r.multVec(V(0, 1, 0))
        ang = math.degrees(math.atan2(y_now.cross(up).dot(along), y_now.dot(up)))
        slab.rotate(V(0, 0, 0), along, ang)
        slab.translate(c)
        cutter = cutter.cut(slab)
    return target.cut(cutter)


def spline_band(pts_yz, half_w, x_face, depth, samples=80, z_clip=None):
    """Planar band (constant half-width) following a B-spline through (y,z) points, lying on the plane x = x_face
    and extruded `depth` along -X (toward the part). Built as a sampled polygon, so it is always a valid solid.
    z_clip=(z0,z1) trims the band to a height range (the band ends get square cuts there)."""
    c = Part.BSplineCurve()
    c.interpolate([V(0, y, z) for y, z in pts_yz])
    p0, p1 = c.FirstParameter, c.LastParameter
    left, right = [], []
    for i in range(samples + 1):
        t = p0 + (p1 - p0) * i / samples
        p = c.value(t); d = c.tangent(t)[0]
        n = V(0, -d.z, d.y); n.normalize()
        left.append(V(x_face, p.y + n.y * half_w, p.z + n.z * half_w))
        right.append(V(x_face, p.y - n.y * half_w, p.z - n.z * half_w))
    poly = left + list(reversed(right)) + [left[0]]
    face = Part.Face(Part.makePolygon(poly))
    solid = face.extrude(V(-depth, 0, 0))
    if z_clip is not None:
        solid = solid.common(Part.makeBox(depth + 10, 1000, z_clip[1] - z_clip[0], V(x_face - depth - 5, -500, z_clip[0])))
    return solid


def annular_sector(r_in, r_out, a0_deg, a1_deg, h, at=(0, 0, 0), axis=Z, rounded=True):
    """Sector of a ring with optional rounded ends (pill-shaped curved slot)."""
    tube_ = tube(r_out, r_in, h)
    a0, a1 = math.radians(a0_deg), math.radians(a1_deg)
    big = r_out * 3
    wedge = polygon_prism([(0, 0), (big * math.cos(a0), big * math.sin(a0)),
                           (big * math.cos((a0 + a1) / 2), big * math.sin((a0 + a1) / 2)),
                           (big * math.cos(a1), big * math.sin(a1))], h)
    s = tube_.common(wedge)
    if rounded:
        rm, rr = (r_in + r_out) / 2, (r_out - r_in) / 2
        for a in (a0, a1):
            s = s.fuse(cyl(rr, h, (rm * math.cos(a), rm * math.sin(a), 0)))
    if _vec(axis) != Z:
        s = _align_z_to(s, axis)
    s.translate(V(*at))
    return s
