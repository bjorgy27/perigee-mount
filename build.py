"""Perigee gimbal generator. Run with:  freecadcmd -c "import runpy; runpy.run_path('build.py')"
Reads params.json, builds every part, the assembly (joints + simulation), and exports
FCStd / STEP / STL / GLB / BOM into ./out and ./viewer.
"""
import os, sys, json, math, time, struct, shutil, importlib.util, types
ROOT = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
sys.path.insert(0, ROOT)
import FreeCAD as App
import Part, Mesh, MeshPart
from gimbal import parts as PP, geom as G
from gimbal.geom import V, X, Y, Z
import tools_guidoc

OUT = os.path.join(ROOT, "out")
VIEWER = os.path.join(ROOT, "viewer")
T0 = time.time()


def log(*a):
    print("[%6.1fs]" % (time.time() - T0), *a, flush=True)


def ensure_dirs():
    """Recreate every fully generated export folder, so parts that no longer exist in the design do not linger
    (old STLs from earlier revisions were sitting next to the current ones and getting sliced)."""
    for d in ("parts_fcstd", "step", "stl"):
        p = os.path.join(OUT, d)
        shutil.rmtree(p, ignore_errors=True)
        os.makedirs(p, exist_ok=True)
    p = os.path.join(VIEWER, "parts")
    shutil.rmtree(p, ignore_errors=True)
    os.makedirs(p, exist_ok=True)


# ------------------------------------------------------------------ headless import of the simulation module
def load_sim_module():
    from PySide import QtCore, QtGui, QtWidgets
    spec = importlib.util.find_spec("CommandCreateSimulation")
    src = open(spec.origin).read()
    mod = types.ModuleType("CommandCreateSimulation")
    mod.__file__ = spec.origin
    mod.QtCore, mod.QtGui, mod.QtWidgets = QtCore, QtGui, QtWidgets
    exec(compile(src, spec.origin, "exec"), mod.__dict__)
    sys.modules["CommandCreateSimulation"] = mod
    return mod


# ------------------------------------------------------------------ GLB writer (positions + indices, flat shaded in the viewer)
def write_glb(path, shape, lin_defl=0.12, ang_defl=0.3):
    m = MeshPart.meshFromShape(Shape=shape, LinearDeflection=lin_defl, AngularDeflection=ang_defl, Relative=False)
    pts, facets = m.Topology
    if not facets:
        return 0
    import array
    pos = array.array("f")
    mn = [1e30] * 3; mx = [-1e30] * 3
    for p in pts:
        pos.extend((p.x, p.y, p.z))
        for i, c in enumerate((p.x, p.y, p.z)):
            mn[i] = min(mn[i], c); mx[i] = max(mx[i], c)
    idx = array.array("I")
    for f in facets:
        idx.extend(f)
    pos_b = pos.tobytes(); idx_b = idx.tobytes()
    pad = lambda b: b + b"\x00" * ((4 - len(b) % 4) % 4)
    pos_b, idx_b = pad(pos_b), pad(idx_b)
    gltf = {
        "asset": {"version": "2.0", "generator": "perigee-gimbal"},
        "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]}],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": len(pts), "type": "VEC3", "min": mn, "max": mx},
            {"bufferView": 1, "componentType": 5125, "count": len(idx), "type": "SCALAR"}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": len(pos_b), "target": 34962},
            {"buffer": 0, "byteOffset": len(pos_b), "byteLength": len(idx_b), "target": 34963}],
        "buffers": [{"byteLength": len(pos_b) + len(idx_b)}],
    }
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    bin_ = pos_b + idx_b
    total = 12 + 8 + len(js) + 8 + len(bin_)
    with open(path, "wb") as f:
        f.write(b"glTF" + struct.pack("<II", 2, total))
        f.write(struct.pack("<I", len(js)) + b"JSON" + js)
        f.write(struct.pack("<I", len(bin_)) + b"BIN\x00" + bin_)
    return len(facets)


def print_oriented(shape, up):
    """Rotate so world direction `up` points +Z, then drop onto z=0 and centre on XY."""
    s = shape.copy()
    upv = V(*up)
    if (upv - Z).Length > 1e-9:
        r = App.Rotation(upv, Z)
        s.Placement = App.Placement(V(0, 0, 0), r).multiply(s.Placement)
    try:
        bb = s.optimalBoundingBox()
    except Exception:
        bb = s.BoundBox
    s.translate(V(-bb.Center.x, -bb.Center.y, -bb.ZMin))
    return s


# ------------------------------------------------------------------ main
def main():
    ensure_dirs()
    P = json.load(open(os.path.join(ROOT, "params.json")))
    doc = App.newDocument("PerigeeGimbal")
    log("building parts")
    L, reg, hw = PP.build_all(P)
    names = [k for k in reg if not k.startswith("_") or k.startswith("_JM")]
    log("parts built:", len(names))

    # ---- spreadsheet of parameters (documentation + single source when re-running the script)
    sheet = doc.addObject("Spreadsheet::Sheet", "Params")
    sheet.set("A1", "Parameter"); sheet.set("B1", "Value"); sheet.set("C1", "Note")
    row = 2
    for k, v in P.items():
        if k.startswith("_"):
            continue
        sheet.set("A%d" % row, k); sheet.set("B%d" % row, str(v))
        try:
            sheet.setAlias("B%d" % row, k)
        except Exception:
            pass
        row += 1
    row += 1
    sheet.set("A%d" % row, "DERIVED"); row += 1
    derived = {
        "el_axis_z": L.z_el, "deck_z": L.z_deck, "az_drive": "goBILDA Stingray-4 direct, 450 deg", "el_drive": "goBILDA Stingray-9 direct, 200 deg",
        "arm_splits_z": " / ".join("%.0f" % z for z in L.arm_splits),
        "counterweight_reach_at_el90": math.hypot(abs(L.y_cw_end) + P["cw_canister_len"], P["cw_canister_d"] / 2),
        "counterweight_clearance_to_yoke_plate": (L.z_el - L.z_plate1) - math.hypot(abs(L.y_cw_end) + P["cw_canister_len"], P["cw_canister_d"] / 2),
        "dish_rim_lowest_z_at_el0": L.z_el - P["dish_diameter"] / 2,
    }
    for k, v in derived.items():
        sheet.set("A%d" % row, k); sheet.set("B%d" % row, str(v)); row += 1
    sheet.set("A%d" % (row + 1), "Edit params.json and re-run build.py; this sheet is regenerated (not live-linked).")

    # ---- assembly
    import Assembly, JointObject, UtilsAssembly  # noqa
    asm = doc.addObject("Assembly::AssemblyObject", "Gimbal")
    group_names = ["Base", "Head", "Cradle"]
    groups = {}
    for gname in group_names:
        gp = doc.addObject("App::Part", gname)
        gp.Label = gname
        asm.addObject(gp)
        groups[gname] = gp
    feats = {}
    for name in names:
        e = reg[name]
        f = doc.addObject("Part::Feature", name.strip("_").replace("-", "_"))
        f.Label = name
        f.Shape = e["shape"]
        f.addProperty("App::PropertyString", "Filament", "Perigee", "Colour / filament").Filament = e["color"]
        f.addProperty("App::PropertyString", "Notes", "Perigee", "Build notes").Notes = e["notes"]
        f.addProperty("App::PropertyBool", "Printed", "Perigee", "Is a printed part").Printed = bool(e["printed"])
        groups[e["group"]].addObject(f)
        if e.get("hidden"):
            f.Visibility = False
        feats[name] = f
    # ---- hardware: one library feature per fastener kind, App::Link instances inside the moving groups
    hwlib = doc.addObject("App::Part", "Hardware_Library")
    hwlib.Label = "Hardware_Library (masters, hidden)"
    hwlib.Visibility = False
    hw_masters = {}
    for key, k in hw.kinds.items():
        f = doc.addObject("Part::Feature", key)
        f.Label = k["label"]
        f.Shape = k["shape"]
        f.addProperty("App::PropertyString", "Filament", "Perigee", "Colour").Filament = k["color"]
        f.addProperty("App::PropertyBool", "Printed", "Perigee", "Is a printed part").Printed = False
        hwlib.addObject(f)
        f.Visibility = False
        hw_masters[key] = f
    hw_groups = {}
    for i, inst in enumerate(hw.instances):
        gname = inst["group"]
        if gname not in hw_groups:
            hg = doc.addObject("App::Part", "Hardware_" + gname)
            hg.Label = "Hardware_" + gname
            groups[gname].addObject(hg)
            hw_groups[gname] = hg
        lk = doc.addObject("App::Link", "%s_%03d" % (inst["kind"], i))
        lk.LinkedObject = hw_masters[inst["kind"]]
        lk.Placement = inst["placement"]
        lk.Label = "%s #%d" % (hw.kinds[inst["kind"]]["label"], i)
        hw_groups[gname].addObject(lk)
    doc.recompute()
    log("hardware instances:", len(hw.instances), "kinds:", len(hw.kinds))

    jg = UtilsAssembly.getJointGroup(asm)
    ground = jg.newObject("App::FeaturePython", "GroundedJoint")
    JointObject.GroundedJoint(ground, groups["Base"])

    def circ_edge_ref(gname, fname, center, axis):
        f = feats[fname]
        for i, e in enumerate(f.Shape.Edges):
            c = e.Curve
            if c.TypeId == "Part::GeomCircle" and (c.Center - V(*center)).Length < 1e-4 and abs(abs(c.Axis.dot(V(*axis))) - 1) < 1e-6:
                el = "%s.Edge%d" % (f.Name, i + 1)
                return [groups[gname], [el, el]]
        raise RuntimeError("no circular edge on %s at %s" % (fname, center))

    JT = JointObject.JointTypes

    def joint(name, jtype, r1, r2, **props):
        j = jg.newObject("App::FeaturePython", name)
        JointObject.Joint(j, JT.index(jtype))
        j.Reference1 = r1
        j.Reference2 = r2
        for k, v in props.items():
            setattr(j, k, v)
        return j

    zc = L.z_el
    ref = lambda g, n, c, a: circ_edge_ref(g, n, c, a)
    j_az = joint("Az_Revolute", "Revolute", ref("Base", "_JM_Az_Base", (0, 0, L.z_puck0 - 0.25), Z),
                 ref("Head", "_JM_Az_Head", (0, 0, L.z_puck0 - 0.25), Z))
    j_el = joint("El_Revolute", "Revolute", ref("Head", "_JM_El_Head", (0.5, 0, zc), X),
                 ref("Cradle", "_JM_El_Cradle", (0.5, 0, zc), X),
                 EnableAngleMin=True, EnableAngleMax=True, AngleMin=-5.0, AngleMax=185.0)
    gear_joints = []
    doc.recompute()
    rc = asm.solve()
    log("assembly solve ->", rc)
    if rc != 0 and gear_joints:
        log("solver unhappy with gear joints; removing them and re-solving")
        for j in gear_joints:
            doc.removeObject(j.Name)
        doc.recompute()
        rc = asm.solve()
        log("assembly solve (no gear joints) ->", rc)
    # positions must be unchanged by solving (all parts modelled in place)
    for gname, gp in groups.items():
        if gp.Placement.Base.Length > 1e-3 or abs(gp.Placement.Rotation.Angle) > 1e-4:
            log("WARNING: solver moved", gname, gp.Placement)

    # ---- simulation (open Assembly > Simulation in the GUI, press Generate, then Play)
    try:
        CCS = load_sim_module()
        sg = UtilsAssembly.getSimulationGroup(asm)
        sim = sg.newObject("App::FeaturePython", "Track_Pass_Simulation")
        CCS.Simulation(sim)
        sim.aTimeStart = 0.0; sim.bTimeEnd = 20.0; sim.cTimeStepOutput = 0.05; sim.jFramesPerSecond = 20
        m1 = asm.newObject("App::FeaturePython", "Motion_Azimuth"); CCS.Motion(m1, "Angular", j_az, "18*time")
        m2 = asm.newObject("App::FeaturePython", "Motion_Elevation"); CCS.Motion(m2, "Angular", j_el, "90 + 90*sin(0.3142*time - 1.5708)")
        sim.Group = [m1, m2]
        log("simulation object created")
    except Exception as ex:
        log("simulation skipped:", ex)

    # ---- geometry + interference audit (validity, watertight meshes, part/part, fastener/part, fastener/fastener)
    audit_geometry(reg, hw)
    audit_overlaps(reg, hw)

    # ---- FreeCAD exploded view (Assembly workbench) following the assembly order
    try:
        make_exploded_view(doc, asm, groups, feats, reg)
    except Exception as ex:
        import traceback; traceback.print_exc()
        log("exploded view skipped:", ex)

    doc.recompute()
    master = os.path.join(OUT, "PerigeeGimbal.FCStd")
    doc.saveAs(master)
    # a headless save has no GuiDocument.xml and FreeCAD's GUI then opens everything hidden: add one (visibility + colours)
    tools_guidoc.inject(master)
    log("saved", master)

    # ---- per-part files + STL + GLB + BOM
    export_parts(doc, feats, reg, P, L)
    export_print_bundle(reg)
    export_hardware(hw)
    write_bom(reg, P, L, hw)
    write_viewer_meta(reg, P, L, hw)
    log("done")


def audit_geometry(reg, hw):
    """Every part must be one valid closed solid that meshes watertight (what the slicer checks), and pass OCC's
    BOP argument check (what FreeCAD's Check Geometry reports). Writes out/geometry_audit.txt."""
    problems = []
    items = [(n, e["shape"], e["printed"]) for n, e in reg.items() if not n.startswith("_")]
    items += [("HW_" + k, v["shape"], False) for k, v in hw.kinds.items()]
    for name, s, printed in items:
        pr = []
        if len(s.Solids) != 1:
            if name != "Feed_Reference":                       # the feed placeholder is three separate rods on purpose
                pr.append("solids=%d" % len(s.Solids))
        if not s.isValid():
            pr.append("invalid")
        if any(not sh.isClosed() for sh in s.Shells):
            pr.append("open shell")
        try:
            s.check(True)
        except Exception as ex:
            pr.append("BOP: " + str(ex).strip().replace("\n", " ")[:120])
        try:
            m = MeshPart.meshFromShape(Shape=s, LinearDeflection=0.05, AngularDeflection=0.25, Relative=False)
            if not m.isSolid():
                pr.append("mesh not watertight")
            if m.hasNonManifolds():
                pr.append("mesh non-manifold")
            if m.hasSelfIntersections():
                pr.append("mesh self-intersecting")
            if m.countComponents() != 1 and printed:
                pr.append("mesh components=%d (enclosed void or separate piece)" % m.countComponents())
        except Exception as ex:
            pr.append("mesh failed: %s" % str(ex)[:80])
        if pr:
            problems.append((name, "; ".join(pr)))
    for name, msg in problems:
        log("GEOMETRY %-26s %s" % (name, msg))
    if not problems:
        log("geometry audit: clean (%d shapes)" % len(items))
    open(os.path.join(OUT, "geometry_audit.txt"), "w").write("\n".join("%s: %s" % p for p in problems) or "clean")
    return problems


def audit_overlaps(reg, hw, min_vol=0.5):
    """Report shared volume between any two parts, between every fastener and every part (a screw shank or nut
    inside plastic means a missing hole, pocket or slot), and between fasteners. Writes out/overlap_audit.txt."""
    names = [n for n, e in reg.items() if not n.startswith("_") and n not in ("Dish_Reference_1m", "Feed_Reference") and not e.get("hidden")]

    def common_vol(sa, sb):
        if not sa.BoundBox.intersect(sb.BoundBox):
            return 0.0
        try:
            return sa.common(sb).Volume
        except Exception:
            return -1.0

    hits = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            v = common_vol(reg[a]["shape"], reg[b]["shape"])
            if v > min_vol or v < 0:
                hits.append((a, b, v))
    placed = []
    for i, inst in enumerate(hw.instances):
        s = hw.kinds[inst["kind"]]["shape"].copy()
        s.Placement = inst["placement"]
        placed.append(("%s#%d" % (inst["kind"], i), s))
    for hn, hs in placed:
        for n in names:
            v = common_vol(hs, reg[n]["shape"])
            if v > min_vol or v < 0:
                hits.append((hn, n, v))
    for i, (ha, sa) in enumerate(placed):
        for hb, sb in placed[i + 1:]:
            v = common_vol(sa, sb)
            if v > min_vol or v < 0:
                hits.append((ha, hb, v))
    if hits:
        for a, b, v in sorted(hits, key=lambda h: -h[2]):
            log("OVERLAP  %-26s %-26s %9.1f mm3" % (a, b, v))
    else:
        log("overlap audit: clean (%d parts, %d fasteners)" % (len(names), len(placed)))
    open(os.path.join(OUT, "overlap_audit.txt"), "w").write("\n".join("%s %s %.1f" % h for h in hits) or "clean")
    return hits


def make_exploded_view(doc, asm, groups, feats, reg):
    from PySide import QtCore, QtGui, QtWidgets
    spec = importlib.util.find_spec("CommandCreateView")
    src = open(spec.origin).read()
    mod = types.ModuleType("CommandCreateView"); mod.__file__ = spec.origin
    mod.QtCore, mod.QtGui, mod.QtWidgets = QtCore, QtGui, QtWidgets
    exec(compile(src, spec.origin, "exec"), mod.__dict__)
    sys.modules["CommandCreateView"] = mod
    vg = None
    for o in asm.Group:
        if o.TypeId == "Assembly::ViewGroup":
            vg = o
    if vg is None:
        vg = asm.newObject("Assembly::ViewGroup", "Exploded Views")
    ev = vg.newObject("App::FeaturePython", "Exploded_Assembly")
    mod.ExplodedView(ev)
    approach = dict(ASSEMBLY_ORDER)
    n = 0
    for name, vec in ASSEMBLY_ORDER:
        if name not in feats or name == "Base_Cup":
            continue
        f = feats[name]; g = groups[reg[name]["group"]]
        st = ev.newObject("App::FeaturePython", "Move_" + name)
        mod.ExplodedViewStep(st, 0)
        try:
            st.References = (g, ["%s." % f.Name])
        except Exception:
            st.References = [g, ["%s." % f.Name]]
        d = 260 if reg[name]["printed"] else 200
        st.MovementTransform = App.Placement(V(vec[0] * d, vec[1] * d, vec[2] * d), App.Rotation())
        n += 1
    doc.recompute()
    log("exploded view with", n, "moves (open Assembly > Exploded_Assembly in the GUI and drag the slider)")


def export_parts(doc, feats, reg, P, L):
    manifest = []
    for name, e in reg.items():
        if name.startswith("_"):
            continue
        shp = e["shape"]
        safe = name
        # GLB in world coordinates for the viewer
        tris = write_glb(os.path.join(VIEWER, "parts", safe + ".glb"), shp)
        rec = dict(name=name, group=e["group"], color=e["color"], printed=e["printed"], qty=e["qty"],
                   hidden=bool(e.get("hidden")), tris=tris, notes=e["notes"], volume_cm3=round(shp.Volume / 1000, 1))
        if e["printed"]:
            po = print_oriented(shp, e["print_up"])
            try:
                bb = po.optimalBoundingBox()
            except Exception:
                bb = po.BoundBox
            rec["print_bbox_mm"] = [round(bb.XLength, 1), round(bb.YLength, 1), round(bb.ZLength, 1)]
            m = MeshPart.meshFromShape(Shape=po, LinearDeflection=0.05, AngularDeflection=0.25, Relative=False)
            m.write(os.path.join(OUT, "stl", safe + ".stl"))
            Part.export([feats[name]], os.path.join(OUT, "step", safe + ".step"))
            pd = App.newDocument("p_" + safe[:20])
            pf = pd.addObject("Part::Feature", safe)
            pf.Shape = po
            pf.Label = name
            pf.addProperty("App::PropertyString", "Filament", "Perigee", "Colour / filament").Filament = e["color"]
            pd.recompute()
            ppath = os.path.join(OUT, "parts_fcstd", safe + ".FCStd")
            pd.saveAs(ppath)
            App.closeDocument(pd.Name)
            tools_guidoc.inject(ppath)
        manifest.append(rec)
    json.dump(manifest, open(os.path.join(OUT, "manifest.json"), "w"), indent=1)
    log("exported", len(manifest), "parts")


def export_print_bundle(reg):
    """out/print: one folder per filament colour with that colour's print-oriented STLs, plus one 3MF per colour holding
    all of them (drop it on a Bambu Studio plate and Arrange)."""
    out = os.path.join(OUT, "print")
    shutil.rmtree(out, ignore_errors=True)
    by_col = {}
    for name, e in reg.items():
        if not name.startswith("_") and e["printed"]:
            by_col.setdefault(e["color"], []).append(name)
    for col, names in sorted(by_col.items()):
        d = os.path.join(out, col); os.makedirs(d)
        pd = App.newDocument("plate_" + col)
        objs = []
        for name in names:
            src = os.path.join(OUT, "stl", name + ".stl")
            shutil.copy(src, os.path.join(d, name + ".stl"))
            o = pd.addObject("Mesh::Feature", name); o.Mesh = Mesh.Mesh(src); o.Label = name; objs.append(o)
        pd.recompute()
        Mesh.export(objs, os.path.join(out, "Perigee_%s_parts.3mf" % col))
        App.closeDocument(pd.Name)
    open(os.path.join(out, "README.txt"), "w").write(
        "PERIGEE gimbal print files. One folder per filament colour with every printed part's STL in print orientation, and one 3MF per\n"
        "colour with all of that colour's parts (open in Bambu Studio, Arrange, split over plates if needed; 256 x 256 bed).\n"
        "ASA or PETG, 0.2 mm layers, 4-5 walls, 30-40 % gyroid for structural parts. Print one dish adapter only.\n")
    log("print bundle:", ", ".join("%s x%d" % (c, len(n)) for c, n in sorted(by_col.items())))


def export_hardware(hw):
    for key, k in hw.kinds.items():
        write_glb(os.path.join(VIEWER, "parts", "HW_" + key + ".glb"), k["shape"], lin_defl=0.08, ang_defl=0.5)
    log("hardware kinds exported:", len(hw.kinds))


def write_bom(reg, P, L, hw):
    dens = P["filament_density_g_cc"]
    lines = ["# Perigee gimbal - bill of materials", "",
             "Generated by build.py from params.json. Filament mass assumes ~45% effective density (walls + infill); treat as +-30%.", "",
             "## Printed parts", "", "| Part | Qty | Filament | Solid volume cm3 | Est. mass g | Print bbox mm | Notes |", "|---|---|---|---|---|---|---|"]
    tot = 0.0
    for name, e in reg.items():
        if name.startswith("_") or not e["printed"]:
            continue
        vol = e["shape"].Volume / 1000.0
        g = vol * dens * 0.45
        tot += g * e["qty"]
        try:
            po = print_oriented(e["shape"], e["print_up"]).optimalBoundingBox()
        except Exception:
            po = print_oriented(e["shape"], e["print_up"]).BoundBox
        lines.append("| %s | %d | %s | %.0f | %.0f | %.0f x %.0f x %.0f | %s |" % (
            name, e["qty"], e["color"], vol, g, po.XLength, po.YLength, po.ZLength, e["notes"].replace("|", "/")))
    lines += ["", "Estimated total filament: **%.0f g**" % tot, ""]
    lines += ["## Fasteners (every one is modelled in the assembly)", "", "| Item | Qty |", "|---|---|"]
    for lbl, n in hw.summary():
        lines.append("| %s | %d |" % (lbl, n))
    lines += ["", "All socket head cap screws are ISO 4762 (DIN 912) and every one ends in an ISO 4032 hex nut in a printed pocket or side-loaded trap; nothing threads into plastic. Tools: 2.5 and 3 mm hex keys, 4 mm for M5, 5.5 / 7 / 8 mm spanners.", ""]
    lines += ["## Other hardware", "",
              "| Item | Qty | Notes |", "|---|---|---|",
              "| PTFE or lithium grease | 1 | AZ thrust face + journal, EL bushings, drive-shaft bushing, gear teeth |",
              "| goBILDA Stingray-4 servo gearbox, 3215-0001-0004 (feedback mode) | 1 | AZ direct drive under the deck, 450 deg, 100 kg.cm |",
              "| goBILDA Stingray-9 servo gearbox, 3215-0001-0009 (feedback mode) | 1 | EL direct drive in the right shoulder, 200 deg, 227 kg.cm |",
              "| AS5600 magnetic encoder board | 1 | left shoulder cap (absolute EL); AZ position comes from the Stingray feedback wire |",
              "| 6 x 2.5 mm diametric magnet | 1 | left stub end |",
              "| Cable service loop (450 deg AZ needs no slip ring) | 1 | through the puck hole into the pedestal annulus |",
              "| Arduino Uno | 1 | in the electronics bay |",
              "| HC-05 / HC-06 Bluetooth serial module | 1 | Easycomm II over the BT link |",
              "| Buck converter 12-24 V -> 5 V | 1 | Uno + encoders + BT |",
              "| M6 x 20 bolts | 4 | base floor to your future tripod / mast plate |",
              "| Sand / steel shot / lead shot | 1.3 L | counterweight canister fill (steel shot ~6 kg) |", ""]
    lines += ["## Key numbers", "",
              "- AZ: Stingray-4 direct drive, 450 deg travel (1.25 turns), 15 rpm, 100 kg.cm. EL: Stingray-9 direct drive, 200 deg travel, 6.6 rpm, 227 kg.cm.",
              "- EL axis %.0f mm above the base bottom, %.0f mm above the deck." % (L.z_el, L.z_el - L.z_deck),
              "- Dish rim lowest point at EL 0/180: z = %.0f mm (base top is %d mm)." % (L.z_el - P["dish_diameter"] / 2, P["base_h"]),
              "- Counterweight reach at EL 90: %.0f mm; yoke plate clearance %.0f mm." % (
                  math.hypot(abs(L.y_cw_end) + P["cw_canister_len"], P["cw_canister_d"] / 2),
                  (L.z_el - L.z_plate1) - math.hypot(abs(L.y_cw_end) + P["cw_canister_len"], P["cw_canister_d"] / 2)), ""]
    open(os.path.join(OUT, "BOM.md"), "w").write("\n".join(lines))
    log("BOM written")


# Assembly sequence (starting from the base). (part, approach direction the part arrives from)
ASSEMBLY_ORDER = [
    ("Base_Cup", (0, 0, 1)), ("Az_Stingray4", (0, 0, 1)), ("Az_Stingray4_Output", (0, 0, 1)),
    ("Head_Puck", (0, 0, 1)), ("Az_Retainer_Ring", (0, 0, 1)),
    ("Az_Drum", (0, 0, 1)), ("Logo_Light_Pipe", (0, 0, 1)), ("Yoke_Plate", (0, 0, 1)), ("Yoke_Access_Cover", (0, 0, 1)),
    ("Arm_R_1", (0, 0, 1)), ("Arm_L_1", (0, 0, 1)), ("Arm_R_2", (0, 0, 1)), ("Arm_L_2", (0, 0, 1)),
    ("El_Stingray9", (-1, 0, 0)), ("El_Stingray9_Output", (-1, 0, 0)), ("El_Bushing_L_Lower", (0, 0, 1)),
    ("El_Stub_L", (-1, 0, 0)), ("Cradle_Hub", (0, 0, 1)), ("Cradle_Lid", (0, 0, 1)),
    ("El_Bushing_L_Upper", (0, 0, 1)), ("El_Bearing_Cap_R", (0, 0, 1)), ("El_Bearing_Cap_L", (0, 0, 1)),
    ("El_Servo_Cover", (1, 0, 0)), ("El_Axis_Cap_L", (-1, 0, 0)), ("Status_Window", (-1, 0, 0)),
    ("Dish_Boom", (0, 1, 0)), ("Dish_Adapter_Plate", (0, 1, 0)),
    ("Counterweight_Arm", (0, -1, 0)), ("Counterweight_Canister", (0, -1, 0)), ("Counterweight_Cap", (0, -1, 0)),
    ("Dish_Reference_1m", (0, 1, 0)), ("Feed_Reference", (0, 1, 0)),
]
SAME_STEP = {"Feed_Reference", "El_Stub_L", "Az_Stingray4_Output", "El_Stingray9_Output"}

SHOTS = [
    ("Pedestal", ["Base_Cup"], ((0, 0, 35), 165, -50, 30, 25), None),
    ("Stingray-4 azimuth drive", ["Az_Stingray4", "Az_Stingray4_Output"], ((0, 20, 35), 130, -60, 45, 30), None),
    ("Deck and flange plate", ["Head_Puck", "Az_Retainer_Ring"], ((0, 0, 70), 170, -40, 26, 30), None),
    ("Azimuth housing, yoke plate, access cover", ["Az_Drum", "Logo_Light_Pipe", "Yoke_Plate", "Yoke_Access_Cover"], ((0, 0, 145), 210, -120, 26, 40), None),
    ("Yoke arms, lower segments", ["Arm_R_1", "Arm_L_1"], ((0, 0, 305), 250, -30, 16, 40), None),
    ("Shoulder segments", ["Arm_R_2", "Arm_L_2"], ((0, 0, 520), 260, -45, 20, 40), None),
    ("Yoke complete", [], ((0, 0, 350), 430, -45, 22, 360), 5.0),
    ("Stingray-9 elevation drive, left bushing", ["El_Stingray9", "El_Stingray9_Output", "El_Bushing_L_Lower"], ((60, 0, 570), 200, -40, 25, 35), None),
    ("Cradle: stub, hub, lid", ["El_Stub_L", "Cradle_Hub", "Cradle_Lid"], ((0, 0, 605), 200, -55, 40, 35), None),
    ("Pillow-block caps", ["El_Bushing_L_Upper", "El_Bearing_Cap_R", "El_Bearing_Cap_L"], ((0, 0, 610), 190, -35, 30, 35), None),
    ("Right shoulder: servo cover", ["El_Servo_Cover"], ((95, 0, 560), 170, -25, 18, 40), None),
    ("Left shoulder: encoder cap and window", ["El_Axis_Cap_L", "Status_Window"], ((-95, 0, 590), 150, -155, 18, 35), None),
    ("Dish boom and adapter", ["Dish_Boom", "Dish_Adapter_Plate"], ((0, 140, 600), 210, 70, 32, 40), None),
    ("Counterweight", ["Counterweight_Arm", "Counterweight_Canister", "Counterweight_Cap"], ((0, -250, 600), 230, -120, 20, 35), None),
    ("1 m dish", ["Dish_Reference_1m", "Feed_Reference"], ((0, 100, 580), 750, -60, 20, 40), None),
    ("PERIGEE gimbal complete", [], ((0, 0, 500), 660, -60, 16, 360), 8.0),
]
PART_STAGGER, SUB_STAGGER, PART_DUR, HW_DUR, HOLD, MIN_SHOT, BLEND = 0.55, 0.18, 0.8, 0.6, 0.7, 2.6, 0.7


def assembly_sequence(reg, hw):
    """Timed items grouped into camera shots. Fasteners follow the part they join."""
    placed = set()
    order = []          # (name, shot index, t0)
    shots = []
    t = 0.0
    for si, (label, plist, cam, dur) in enumerate(SHOTS):
        plist = [n for n in plist if n in reg]
        t_start = t
        t_item = t_start + BLEND * 0.6
        last_end = t_start
        for j, n in enumerate(plist):
            sub = si == 2 and j > 0      # head module arrives as one pre-assembled unit
            order.append((n, si, round(t_item, 3)))
            placed.add(n)
            last_end = max(last_end, t_item + PART_DUR + HW_DUR + 0.5)
            t_item += SUB_STAGGER if sub else PART_STAGGER
        t_end = max(t_start + (dur or MIN_SHOT), last_end + HOLD)
        shots.append(dict(label=label, t0=round(t_start, 3), t1=round(t_end, 3), center=list(cam[0]), extent=cam[1],
                          theta=cam[2], elev=cam[3], sweep=cam[4]))
        t = t_end
    # anything not listed goes into the last placing shot
    for n in reg:
        if not n.startswith("_") and n not in placed:
            order.append((n, len(SHOTS) - 2, shots[-2]["t0"] + 1.0))
    approach = dict(ASSEMBLY_ORDER)
    names = [o[0] for o in order]
    idx_of = {n: i for i, (n, _, _) in enumerate(order)}
    bbs = {}
    for n in names:
        bb = reg[n]["shape"].BoundBox
        bbs[n] = (bb.XMin - 1, bb.YMin - 1, bb.ZMin - 1, bb.XMax + 1, bb.YMax + 1, bb.ZMax + 1)
    items = []
    for n, si, t0 in order:
        items.append(dict(kind="part", name=n, shot=si, t0=t0, dur=PART_DUR,
                          dist=(320 if reg[n]["printed"] or n.startswith("Dish") else 200), approach=list(approach.get(n, (0, 0, 1)))))
    counts = {}
    for i, inst in enumerate(hw.instances):
        shp = hw.kinds[inst["kind"]]["shape"].copy()
        shp.Placement = inst["placement"]
        b = shp.BoundBox
        touching = [n for n in names if reg[n]["printed"] and not (b.XMax < bbs[n][0] or b.XMin > bbs[n][3] or b.YMax < bbs[n][1] or b.YMin > bbs[n][4] or b.ZMax < bbs[n][2] or b.ZMin > bbs[n][5])]
        host = max(touching, key=lambda n: idx_of[n]) if touching else names[0]
        hn, hs, ht0 = order[idx_of[host]]
        j = counts.get(host, 0); counts[host] = j + 1
        items.append(dict(kind="hw", index=i, shot=hs, t0=round(ht0 + PART_DUR * 0.6 + 0.05 * j, 3), dur=HW_DUR, dist=60))
    total = shots[-1]["t1"]
    return dict(items=items, shots=shots, total=round(total, 2), blend=BLEND, steps=len([s for s in SHOTS if s[1]]))


def _m4(plc):
    m = plc.toMatrix()
    return [[m.A11, m.A12, m.A13, m.A14], [m.A21, m.A22, m.A23, m.A24], [m.A31, m.A32, m.A33, m.A34], [0, 0, 0, 1]]


def write_viewer_meta(reg, P, L, hw):
    meta = {
        "title": "PERIGEE gimbal",
        "params": {k: v for k, v in P.items() if not k.startswith("_")},
        "kinematics": {
            "az_axis": {"origin": [0, 0, L.z_puck0], "axis": [0, 0, 1]},
            "el_axis": {"origin": [0, 0, L.z_el], "axis": [1, 0, 0]},
            "gears": {},
        },
        "colors": {k: list(v) for k, v in PP.COLORS.items()},
        "parts": [dict(name=n, file="parts/%s.glb" % n, group=e["group"], color=e["color"], printed=e["printed"],
                       hidden=bool(e.get("hidden")), notes=e["notes"]) for n, e in reg.items() if not n.startswith("_")],
        "hardware_kinds": {k: dict(file="parts/HW_%s.glb" % k, color=v["color"], label=v["label"]) for k, v in hw.kinds.items()},
        "hardware": [dict(kind=i["kind"], group=i["group"], matrix=[list(row) for row in _m4(i["placement"])]) for i in hw.instances],
        "assembly": assembly_sequence(reg, hw),
    }
    json.dump(meta, open(os.path.join(VIEWER, "meta.json"), "w"), indent=1)
    log("viewer meta written")


main()
