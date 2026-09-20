"""Perigee gimbal generator. Run with:  freecadcmd -c "import runpy; runpy.run_path('build.py')"
Reads params.json, builds every part, the assembly (joints + simulation), TechDraw pages, and exports
FCStd / STEP / STL / GLB / BOM into ./out and ./viewer.
"""
import os, sys, json, math, time, struct, shutil, importlib.util, types
ROOT = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
sys.path.insert(0, ROOT)
import FreeCAD as App
import Part, Mesh, MeshPart
from gimbal import parts as PP, geom as G
from gimbal.geom import V, X, Y, Z

OUT = os.path.join(ROOT, "out")
VIEWER = os.path.join(ROOT, "viewer")
T0 = time.time()


def log(*a):
    print("[%6.1fs]" % (time.time() - T0), *a, flush=True)


def ensure_dirs():
    for d in ("parts_fcstd", "step", "stl", "drawings", "viewer_parts"):
        os.makedirs(os.path.join(OUT, d), exist_ok=True)
    os.makedirs(os.path.join(VIEWER, "parts"), exist_ok=True)


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
def write_glb(path, shape, lin_defl=0.25, ang_defl=0.4):
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
        "el_axis_z": L.z_el, "deck_z": L.z_deck, "az_ratio": P["az_ring_teeth"] / P["az_pinion_teeth"],
        "el_ratio": P["el_gear_teeth"] / P["el_pinion_teeth"], "az_pinion_centre_y": L.az_pinion_c[1],
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
    group_names = ["Base", "Head", "Cradle", "AzPinion", "ElPinion", "Idler", "Sensor"]
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
    px, py = L.az_pinion_c
    gear_joints = []
    try:
        j1 = joint("AzPinion_Revolute", "Revolute", ref("Head", "_JM_AzPin_Head", (px, py, L.z_ring0 + 4.5), Z),
                   ref("AzPinion", "_JM_AzPin_Pin", (px, py, L.z_ring0 + 4.5), Z))
        j2 = joint("ElPinion_Revolute", "Revolute", ref("Head", "_JM_ElPin_Head", (L.x_gear0 + 3.5, 0, L.z_elpin), X),
                   ref("ElPinion", "_JM_ElPin_Pin", (L.x_gear0 + 3.5, 0, L.z_elpin), X))
        ix, iy = L.idler_c; gx, gy = L.sens_c
        j3 = joint("Idler_Revolute", "Revolute", ref("Head", "_JM_Idler_Head", (ix, iy, L.z_sens0 + 3.5), Z),
                   ref("Idler", "_JM_Idler_Idler", (ix, iy, L.z_sens0 + 3.5), Z))
        j4 = joint("Sensor_Revolute", "Revolute", ref("Head", "_JM_Sens_Head", (gx, gy, L.z_sens0 + 3.5), Z),
                   ref("Sensor", "_JM_Sens_Sens", (gx, gy, L.z_sens0 + 3.5), Z))
        g1 = joint("Az_Gears", "Gears", ref("AzPinion", "_JM_AzPin_Pin", (px, py, L.z_ring0 + 4.5), Z),
                   ref("Base", "_JM_Ring_Base", (0, 0, L.z_ring0 + 4.5), Z), Distance=L.r_azpin, Distance2=L.r_ring)
        g2 = joint("El_Gears", "Gears", ref("ElPinion", "_JM_ElPin_Pin", (L.x_gear0 + 3.5, 0, L.z_elpin), X),
                   ref("Cradle", "_JM_ElGear_Cradle", (L.x_gear0 + 3.5, 0, zc), X), Distance=L.r_elpin, Distance2=L.r_elgear)
        g3 = joint("Idler_Gears", "Gears", ref("Idler", "_JM_Idler_Idler", (ix, iy, L.z_sens0 + 3.5), Z),
                   ref("Base", "_JM_Sun_Base", (0, 0, L.z_sens0 + 3.5), Z), Distance=L.r_idler, Distance2=L.r_sun)
        g4 = joint("Sensor_Gears", "Gears", ref("Sensor", "_JM_Sens_Sens", (gx, gy, L.z_sens0 + 3.5), Z),
                   ref("Idler", "_JM_Idler_Idler", (ix, iy, L.z_sens0 + 3.5), Z), Distance=L.r_sens, Distance2=L.r_idler)
        gear_joints = [j1, j2, j3, j4, g1, g2, g3, g4]
    except Exception as ex:
        log("gear joints skipped:", ex)
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

    # ---- interference audit (printed parts + placeholders, pairwise)
    audit_overlaps(reg)

    # ---- FreeCAD exploded view (Assembly workbench) following the assembly order
    try:
        make_exploded_view(doc, asm, groups, feats, reg)
    except Exception as ex:
        import traceback; traceback.print_exc()
        log("exploded view skipped:", ex)

    # ---- TechDraw pages
    try:
        make_drawings(doc, feats, reg, P, L)
    except Exception as ex:
        import traceback; traceback.print_exc()
        log("drawings failed:", ex)

    doc.recompute()
    master = os.path.join(OUT, "PerigeeGimbal.FCStd")
    doc.saveAs(master)
    log("saved", master)

    # ---- per-part files + STL + GLB + BOM
    export_parts(doc, feats, reg, P, L)
    export_hardware(hw)
    write_bom(reg, P, L, hw)
    write_viewer_meta(reg, P, L, hw)
    log("done")


def audit_overlaps(reg, min_vol=5.0):
    """Report any two solid parts that share volume (fasteners are checked separately by design of their holes)."""
    names = [n for n, e in reg.items() if not n.startswith("_") and n not in ("Dish_Reference_1m", "Feed_Reference") and not e.get("hidden")]
    hits = []
    for i, a in enumerate(names):
        sa = reg[a]["shape"]; ba = sa.BoundBox
        for b in names[i + 1:]:
            sb = reg[b]["shape"]
            if not ba.intersect(sb.BoundBox):
                continue
            try:
                v = sa.common(sb).Volume
            except Exception:
                v = -1
            if v > min_vol or v < 0:
                hits.append((a, b, v))
    if hits:
        for a, b, v in sorted(hits, key=lambda h: -h[2]):
            log("OVERLAP  %-26s %-26s %9.0f mm3" % (a, b, v))
    else:
        log("overlap audit: clean (%d parts)" % len(names))
    open(os.path.join(OUT, "overlap_audit.txt"), "w").write("\n".join("%s %s %.0f" % h for h in hits) or "clean")
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


def make_drawings(doc, feats, reg, P, L):
    import TechDraw
    tmpl = "/usr/share/freecad/Mod/TechDraw/Templates/ISO/A3_Landscape_TD.svg"
    sheets = {
        "01_Base": ["Base_Cup", "Az_Ring_Gear", "Sensor_Sun_Gear", "Az_Retainer_Ring"],
        "02_Head_Puck": ["Head_Puck"],
        "03_Arms_Right": ["Arm_R_1", "Arm_R_2", "Arm_R_3"],
        "04_Arms_Left": ["Arm_L_1", "Arm_L_2", "Arm_L_3"],
        "05_Cradle": ["Cradle_Hub", "Cradle_Lid", "El_Stub_R", "El_Stub_L", "El_Bearing_Cap_R", "El_Bushing_R_Lower"],
        "06_Gears": ["El_Gear", "El_Pinion", "Az_Pinion", "Sensor_Idler", "Sensor_Gear"],
        "07_Boom_Counterweight": ["Dish_Boom", "Counterweight_Arm", "Counterweight_Canister", "Counterweight_Cap"],
        "08_Housings": ["Az_Drum", "Yoke_Plate", "Yoke_Access_Cover", "El_Servo_Cover", "El_Axis_Cap_R", "El_Axis_Cap_L"],
        "09_Adapters_Small": ["Dish_Adapter_Plate", "Dish_Adapter_PipeStub", "SlipRing_Stub_Clamp", "Az_Servo_Clamp", "Sensor_Gear_Keeper", "Status_Window"],
    }
    pages = []
    for pname, plist in sheets.items():
        pg = doc.addObject("TechDraw::DrawPage", "Page_" + pname)
        tp = doc.addObject("TechDraw::DrawSVGTemplate", "Tmpl_" + pname)
        tp.Template = tmpl
        pg.Template = tp
        n = len(plist)
        cols = min(n, 3)
        rows = int(math.ceil(n / cols))
        cw, rh = 400 / cols, 250 / rows
        plist = [p for p in plist if p in feats]
        n = len(plist)
        for i, part in enumerate(plist):
            f = feats[part]
            bb = f.Shape.BoundBox
            big = max(bb.XLength, bb.YLength, bb.ZLength)
            scale = min(1.0, (min(cw, rh) * 0.38) / max(big, 1))
            cx = 20 + cw * (i % cols) + cw / 2
            cy = 40 + rh * (i // cols) + rh / 2
            # front (looking along -Y), top (looking down), right (looking along -X) and iso
            for k, (d, xd, dx, dy) in enumerate([((0, -1, 0), (1, 0, 0), -cw * 0.24, 0), ((0, 0, 1), (1, 0, 0), -cw * 0.24, rh * 0.42),
                                                  ((1, 0, 0), (0, 1, 0), cw * 0.16, 0), ((1, -1, 1), (1, 1, 0), cw * 0.24, rh * 0.42)]):
                v = doc.addObject("TechDraw::DrawViewPart", "V_%s_%s_%d" % (pname, part, k))
                v.Source = [f]
                v.Direction = V(*d)
                v.XDirection = V(*xd)
                v.ScaleType = "Custom"
                v.Scale = scale
                v.X = cx + dx
                v.Y = 297 - (cy + dy)
                v.HardHidden = False
                pg.addView(v)
            a = doc.addObject("TechDraw::DrawViewAnnotation", "A_%s_%s" % (pname, part))
            a.Text = ["%s  (%s)" % (part, reg[part]["color"]),
                      "%.0f x %.0f x %.0f mm  scale 1:%.1f" % (bb.XLength, bb.YLength, bb.ZLength, 1 / scale),
                      reg[part]["notes"][:90]]
            a.TextSize = 3.5
            a.X = cx - cw * 0.45
            a.Y = 297 - (cy - rh * 0.45)
            pg.addView(a)
        pages.append(pg)
    doc.recompute()
    for pg in pages:
        try:
            TechDraw.writeDXFPage(pg, os.path.join(OUT, "drawings", pg.Name.replace("Page_", "") + ".dxf"))
        except Exception as ex:
            log("dxf export failed for", pg.Name, ex)
    log("drawings:", len(pages), "pages")


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
            pd.recompute()
            pd.saveAs(os.path.join(OUT, "parts_fcstd", safe + ".FCStd"))
            App.closeDocument(pd.Name)
        manifest.append(rec)
    json.dump(manifest, open(os.path.join(OUT, "manifest.json"), "w"), indent=1)
    log("exported", len(manifest), "parts")


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
              "| Servos, ~%dx%dx%d mm box, high torque | 2 | AZ must be continuous-rotation; EL continuous or >= 300 deg travel with the %.1f:1 ratio |" % (
                  P["servo_w"], P["servo_h"], P["servo_len"], P["el_gear_teeth"] / P["el_pinion_teeth"]),
              "| AS5600 magnetic encoder boards | 2 | one under the deck (AZ), one in the left trunnion cap (EL) |",
              "| 6 x 2.5 mm diametric magnets | 2 | AZ sensor gear pin + left trunnion end |",
              "| 22 mm capsule slip ring, 6-12 wire | 1 | power up to the head; parallel wires for servo current |",
              "| Arduino Uno | 1 | in the electronics bay |",
              "| HC-05 / HC-06 Bluetooth serial module | 1 | Easycomm II over the BT link |",
              "| Buck converter 12-24 V -> 5 V | 1 | Uno + encoders + BT |",
              "| M6 x 20 bolts | 4 | base floor to your future tripod / mast plate |",
              "| Sand / steel shot / lead shot | 1.3 L | counterweight canister fill (steel shot ~6 kg) |", ""]
    lines += ["## Key numbers", "",
              "- AZ ratio %.2f:1 (ring %d T / pinion %d T, module %g). EL ratio %.1f:1 (gear %d T / pinion %d T)." % (
                  P["az_ring_teeth"] / P["az_pinion_teeth"], P["az_ring_teeth"], P["az_pinion_teeth"], P["gear_module"],
                  P["el_gear_teeth"] / P["el_pinion_teeth"], P["el_gear_teeth"], P["el_pinion_teeth"]),
              "- AZ encoder train: sun %d T (fixed) -> idler %d T -> sensor %d T = exactly 1 turn per turn." % (
                  P["sensor_sun_teeth"], P["sensor_idler_teeth"], P["sensor_gear_teeth"]),
              "- EL axis %.0f mm above the base bottom, %.0f mm above the deck." % (L.z_el, L.z_el - L.z_deck),
              "- Dish rim lowest point at EL 0/180: z = %.0f mm (base top is %d mm)." % (L.z_el - P["dish_diameter"] / 2, P["base_h"]),
              "- Counterweight reach at EL 90: %.0f mm; yoke plate clearance %.0f mm." % (
                  math.hypot(abs(L.y_cw_end) + P["cw_canister_len"], P["cw_canister_d"] / 2),
                  (L.z_el - L.z_plate1) - math.hypot(abs(L.y_cw_end) + P["cw_canister_len"], P["cw_canister_d"] / 2)), ""]
    open(os.path.join(OUT, "BOM.md"), "w").write("\n".join(lines))
    log("BOM written")


# Assembly sequence (starting from the base). (part, approach direction the part arrives from)
ASSEMBLY_ORDER = [
    ("Base_Cup", (0, 0, 1)), ("Az_Ring_Gear", (0, 0, 1)), ("Sensor_Sun_Gear", (0, 0, 1)), ("SlipRing_Capsule_22mm", (0, 0, 1)),
    ("Head_Puck", (0, 0, 1)), ("Sensor_Idler", (0, 0, 1)), ("Sensor_Gear", (0, 0, 1)), ("Sensor_Gear_Keeper", (0, 0, 1)),
    ("Az_Pinion", (0, 0, 1)), ("SlipRing_Stub_Clamp", (0, 0, 1)), ("Az_Servo_placeholder", (0, 0, 1)),
    ("Az_Retainer_Ring", (0, 0, 1)), ("Az_Servo_Clamp", (0, 0, 1)),
    ("Az_Drum", (0, 0, 1)), ("Logo_Light_Pipe", (0, 0, 1)), ("Yoke_Plate", (0, 0, 1)), ("Yoke_Access_Cover", (0, 0, 1)),
    ("Arm_R_1", (0, 0, 1)), ("Arm_L_1", (0, 0, 1)), ("Arm_R_2", (0, 0, 1)), ("Arm_L_2", (0, 0, 1)),
    ("El_Bushing_R_Lower", (0, 0, 1)), ("El_Bushing_L_Lower", (0, 0, 1)),
    ("El_Stub_R", (1, 0, 0)), ("El_Stub_L", (-1, 0, 0)), ("El_Gear", (1, 0, 0)), ("Cradle_Hub", (0, 0, 1)), ("Cradle_Lid", (0, 0, 1)),
    ("El_Bushing_R_Upper", (0, 0, 1)), ("El_Bushing_L_Upper", (0, 0, 1)), ("El_Bearing_Cap_R", (0, 0, 1)), ("El_Bearing_Cap_L", (0, 0, 1)),
    ("El_Pinion", (1, 0, 0)), ("El_Servo_placeholder", (1, 0, 0)), ("El_Servo_Cover", (1, 0, 0)), ("El_Axis_Cap_R", (1, 0, 0)),
    ("El_Axis_Cap_L", (-1, 0, 0)), ("Status_Window", (-1, 0, 0)),
    ("Dish_Boom", (0, 1, 0)), ("Dish_Adapter_Plate", (0, 1, 0)),
    ("Counterweight_Arm", (0, -1, 0)), ("Counterweight_Canister", (0, -1, 0)), ("Counterweight_Cap", (0, -1, 0)),
    ("Dish_Reference_1m", (0, 1, 0)), ("Feed_Reference", (0, 1, 0)),
]
SAME_STEP = {"Sensor_Idler", "Sensor_Gear", "Sensor_Gear_Keeper", "Az_Pinion", "SlipRing_Stub_Clamp", "Az_Servo_placeholder", "Feed_Reference",
             "El_Stub_L", "El_Gear"}

SHOTS = [
    ("Pedestal", ["Base_Cup"], ((0, 0, 30), 165, -50, 30, 25), None),
    ("Ring gear, sun gear, slip ring", ["Az_Ring_Gear", "Sensor_Sun_Gear", "SlipRing_Capsule_22mm"], ((0, 0, 25), 120, -60, 55, 30), None),
    ("Head module: deck, encoder train, AZ pinion, servo", ["Head_Puck", "Sensor_Idler", "Sensor_Gear", "Sensor_Gear_Keeper", "Az_Pinion", "SlipRing_Stub_Clamp", "Az_Servo_placeholder"], ((0, 0, 60), 165, -55, 30, 25), None),
    ("Flange plate and servo clamp", ["Az_Retainer_Ring", "Az_Servo_Clamp"], ((0, 0, 60), 170, -40, 26, 30), None),
    ("Azimuth housing, yoke plate, access cover", ["Az_Drum", "Logo_Light_Pipe", "Yoke_Plate", "Yoke_Access_Cover"], ((0, 0, 125), 210, -120, 26, 40), None),
    ("Yoke arms, lower segments", ["Arm_R_1", "Arm_L_1"], ((0, 0, 285), 250, -30, 16, 40), None),
    ("Shoulder segments", ["Arm_R_2", "Arm_L_2"], ((0, 0, 500), 260, -45, 20, 40), None),
    ("Yoke complete", [], ((0, 0, 330), 430, -45, 22, 360), 5.0),
    ("Cradle: bushings, stubs, gear, hub, lid", ["El_Bushing_R_Lower", "El_Bushing_L_Lower", "El_Stub_R", "El_Stub_L", "El_Gear", "Cradle_Hub", "Cradle_Lid"], ((0, 0, 585), 200, -55, 40, 35), None),
    ("Pillow-block caps", ["El_Bushing_R_Upper", "El_Bushing_L_Upper", "El_Bearing_Cap_R", "El_Bearing_Cap_L"], ((0, 0, 590), 190, -35, 30, 35), None),
    ("Right shoulder: pinion, servo, covers", ["El_Pinion", "El_Servo_placeholder", "El_Servo_Cover", "El_Axis_Cap_R"], ((95, 0, 540), 170, -25, 18, 40), None),
    ("Left shoulder: encoder cap and window", ["El_Axis_Cap_L", "Status_Window"], ((-95, 0, 570), 150, -155, 18, 35), None),
    ("Dish boom and adapter", ["Dish_Boom", "Dish_Adapter_Plate"], ((0, 140, 580), 210, 70, 32, 40), None),
    ("Counterweight", ["Counterweight_Arm", "Counterweight_Canister", "Counterweight_Cap"], ((0, -250, 580), 230, -120, 20, 35), None),
    ("1 m dish", ["Dish_Reference_1m", "Feed_Reference"], ((0, 100, 560), 750, -60, 20, 40), None),
    ("PERIGEE gimbal complete", [], ((0, 0, 480), 660, -60, 16, 360), 8.0),
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
            "gears": {
                "AzPinion": {"origin": [L.az_pinion_c[0], L.az_pinion_c[1], 0], "axis": [0, 0, 1], "ratio_vs_az": P["az_ring_teeth"] / P["az_pinion_teeth"]},
                "ElPinion": {"origin": [0, 0, L.z_elpin], "axis": [1, 0, 0], "ratio_vs_el": -P["el_gear_teeth"] / P["el_pinion_teeth"]},
                "Idler": {"origin": [L.idler_c[0], L.idler_c[1], 0], "axis": [0, 0, 1], "ratio_vs_az": P["sensor_sun_teeth"] / P["sensor_idler_teeth"]},
                "Sensor": {"origin": [L.sens_c[0], L.sens_c[1], 0], "axis": [0, 0, 1], "ratio_vs_az": -1.0},
            },
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
