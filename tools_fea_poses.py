"""Bonded, defeatured FEA structure at several elevation poses. Run with:
    freecadcmd -c "import runpy; runpy.run_path('tools_fea_poses.py')"   [EL angles from FEA_POSES env, default "0,45,90"]

Reuses the defeaturing, grouping and bridge solids of tools_fea.py unchanged; only the cradle group (hub, lid, stub,
boom, counterweight arm) and its two spigot bridges are rotated about the EL axis before the fuse, so one mesh per
pose carries the real load geometry. Writes out/fea/sweep/pose_<el>.brep and pose_<el>.json (face centres for the
foot, boom flange and counterweight arm end, found again by geometry in tools_fea_sweep.py)."""
import os, sys, json, math
ROOT = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
sys.path.insert(0, ROOT)
import FreeCAD as App
import Part
from gimbal import geom as G

src = open(os.path.join(ROOT, "tools_fea.py")).read()
src = src[:src.rindex("\nmain()")]                 # everything but the module-level main() call
ns = {"__file__": os.path.join(ROOT, "tools_fea.py"), "__name__": "tools_fea_lib"}
exec(compile(src, "tools_fea.py", "exec"), ns)
build_defeatured, GROUPS, bridges, fuse_all = ns["build_defeatured"], ns["GROUPS"], ns["bridges"], ns["fuse_all"]

OUT = os.path.join(ROOT, "out", "fea", "sweep")
os.makedirs(OUT, exist_ok=True)
V = App.Vector


def rot_el(shape, L, el_deg):
    s = shape.copy()
    s.rotate(V(0, 0, L.z_el), V(1, 0, 0), el_deg)     # +EL tips the boom (+Y) upward
    return s


def pt_el(y, L, el_deg):
    a = math.radians(el_deg)
    return (0.0, y * math.cos(a), L.z_el + y * math.sin(a))


def main():
    P = json.load(open(os.path.join(ROOT, "params.json")))
    els = [float(x) for x in os.environ.get("FEA_POSES", "0,45,90").split(",")]
    L, reg = build_defeatured(P)
    group = {g: fuse_all([reg[n]["shape"] for n in names if n in reg]) for g, names in GROUPS.items()}
    br = bridges(L)
    # the two Y-axis spigot bridges (boom / counterweight arm roots) belong to the cradle; index them by geometry
    cradle_br = [b for b in br if abs(b.BoundBox.ZMin - (L.z_el - 15.5)) < 1 and abs(b.BoundBox.XMin + 15.5) < 1 and b.BoundBox.YLength < 12]
    static_br = [b for b in br if b not in cradle_br]
    print("bridges: %d cradle, %d static" % (len(cradle_br), len(static_br)))
    assert len(cradle_br) == 2
    for el in els:
        parts = [group["base"], group["head"], rot_el(group["cradle"], L, el)] + static_br + [rot_el(b, L, el) for b in cradle_br]
        structure = fuse_all(parts)
        ok = len(structure.Solids) == 1 and structure.isValid()
        print("EL %g: solids=%d valid=%s faces=%d volume=%.0f cm3" % (el, len(structure.Solids), structure.isValid(), len(structure.Faces), structure.Volume / 1000))
        if not ok:
            print("  WARNING: not one valid solid; volumes", sorted((round(s.Volume / 1000) for s in structure.Solids), reverse=True)[:6])
        tag = "%03d" % int(round(el))
        structure.exportBrep(os.path.join(OUT, "pose_%s.brep" % tag))
        meta = {"el_deg": el, "z_el": L.z_el, "y_boom_end": L.y_boom_end, "y_cw_end": L.y_cw_end,
                "foot_center": (0.0, 0.0, 0.0), "dish_center": pt_el(L.y_boom_end, L, el), "cw_center": pt_el(L.y_cw_end, L, el),
                "dish_offset": P["dish_offset"], "volume_cm3": structure.Volume / 1000, "one_solid": ok}
        json.dump(meta, open(os.path.join(OUT, "pose_%s.json" % tag), "w"), indent=1)
        print("  wrote pose_%s.brep/json" % tag)


main()
