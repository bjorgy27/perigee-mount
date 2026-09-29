"""FEA model of the Perigee gimbal. Run with:  freecadcmd -c "import runpy; runpy.run_path('tools_fea.py')"

Builds the printed parts again *defeatured* (no bolt holes, nut traps, counterbores or stencil text: they only make the
mesh huge and do not change the load path), fuses the structural chain into ONE bonded solid, and writes:

  out/fea/Perigee_FEA.FCStd         FreeCAD FEM analysis: PLA material, fixed foot, dish weight, counterweight (on the arm end), wind,
                                    gravity, a gmsh mesh object and a CalculiX static solver, ready to mesh and run
  out/fea/Perigee_FEA_structure.step  the same bonded solid for any other FEA package
  out/fea/Perigee_FEA_<group>.step    base / head / cradle fused separately

Bonded assumptions (first-pass static model, conservative in stiffness where noted):
  - the AZ plain bearing (puck on the pedestal thrust face) is bonded,
  - the Stingray-4 block, its hub and standoffs are PLA bridge solids between pedestal floor and puck,
  - the Stingray-9 block and its hub/standoffs are PLA bridge solids between hub and right shoulder drum
    (real parts are aluminium and steel: the model overestimates deflection there),
  - the left stub axle is bonded into its bushing.
Meshing needs gmsh on PATH (pacman -S gmsh), solving needs CalculiX (AUR: calculix). Open the file, double-click
Mesh, click Apply; then double-click CalculiX, Write input file, Run; then open the result object.
"""
import os, sys, json, math
ROOT = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
sys.path.insert(0, ROOT)
import FreeCAD as App
import Part
from gimbal import parts as PP, geom as G, hardware as HW
import tools_guidoc

OUT = os.path.join(ROOT, "out", "fea")
os.makedirs(OUT, exist_ok=True)
V = App.Vector


class NoCutHardware(HW.Hardware):
    """Registers nothing and cuts nothing: the parts come out without fastener features."""
    def screw(self, group, d, L, at, direction):
        return L

    def nut(self, *a, **k):
        pass

    def washer(self, *a, **k):
        pass

    def insert(self, *a, **k):
        pass

    @staticmethod
    def cbore_hole(shape, *a, **k):
        return shape

    @staticmethod
    def clear_hole(shape, *a, **k):
        return shape

    @staticmethod
    def insert_hole(shape, *a, **k):
        return shape

    @staticmethod
    def nut_trap(shape, *a, **k):
        return shape

    @staticmethod
    def nut_pocket(shape, *a, **k):
        return shape


def far_box(*a, **k):
    return Part.makeBox(0.1, 0.1, 0.1, V(99999, 99999, 99999))


def build_defeatured(P):
    G.stencil_text_cut = lambda target, *a, **k: target
    G.text_solid = far_box
    L = PP.Layout(P); reg = {}; hw = NoCutHardware()
    PP.build_base(P, L, reg, hw); PP.build_arms(P, L, reg, hw); PP.build_head(P, L, reg, hw); PP.build_cradle(P, L, reg, hw)
    return L, reg


GROUPS = {
    "base": ["Base_Cup", "Az_Retainer_Ring_A", "Az_Retainer_Ring_B"],
    "head": ["Head_Puck", "Az_Drum", "Yoke_Plate", "Arm_R_1", "Arm_L_1", "Arm_R_2", "Arm_L_2", "El_Bearing_Cap_R", "El_Bearing_Cap_L",
             "El_Bushing_L_Lower", "El_Bushing_L_Upper"],
    "cradle": ["Cradle_Hub", "Cradle_Lid", "El_Stub_L", "Dish_Boom", "Counterweight_Arm"],   # canister: its load is applied on the arm end instead
}


def bridges(L):
    """PLA stand-ins for the metal load-path pieces and for the bearing contacts, so one fuse gives one solid."""
    S = PP.STINGRAY
    b = []
    # AZ: Stingray-4 block on its pillars, hub + standoffs up to the puck underside, thrust-face bond ring
    b.append(G.box(2 * 21.8, 69.8 + 21.8, 46.8 - 26.4 + 0.4, (-21.8, -21.8, 26.2)))
    b.append(G.cyl(16, L.z_puck0 - 46.8 + 2.5, (0, 0, 46.6)))                      # reaches past the shaft-end recess in the puck (no sealed void)
    b.append(G.tube(L.R_bore, L.R_thrust, 1.0, (0, 0, L.z_puck0 - 0.5)))
    # EL right: Stingray-9 block filling its pocket (slightly oversize so it bonds to the pocket walls), hub + standoffs to the cradle wall
    bx0, bx1 = L.x_el_block[0] - S["clear"] - 0.2, L.x_el_block[1] + S["clear"] + 0.2
    by, bz0, bz1 = S["block"][3] + S["clear"] + 0.2, S["block"][0] - S["clear"] - 0.2, S["block"][1] + S["clear"] + 0.2
    b.append(G.box(bx1 - bx0, 2 * by, bz1 - bz0, (bx0, -by, L.z_el + bz0)))
    b.append(G.cyl(16, bx0 - L.x_el_load + 2.5, (L.x_el_load - 2.3, 0, L.z_el), (1, 0, 0)))   # fills the hub's shaft-end recess too
    # EL left: stub bonded into the bushing halves and the drum bore
    b.append(G.cyl(L.r_bush + 0.4, 42, (-(L.x_arm_in + 42) - 1, 0, L.z_el), (1, 0, 0)))
    # sliding fits closed up, otherwise two surfaces a few tenths apart make the mesher's facets intersect:
    # column spigots in their sockets, boom / counterweight / stub spigots in the hub recesses
    z1 = L.arm_splits[1]
    for sx in (-1, 1):
        b.append(G.tube(L.arm_r - L.arm_wall + 0.15, L.arm_r - L.arm_wall - 0.35, 26, (sx * L.x_arm_mid, 0, z1 - 25)))
    hs = L.hub
    b.append(G.cyl(15.5, 11, (0, hs / 2 - 10.5, L.z_el), (0, 1, 0)))
    b.append(G.cyl(15.5, 11, (0, -(hs / 2 - 10.5), L.z_el), (0, -1, 0)))
    b.append(G.cyl(15.5, 7, (-(hs / 2 + 0.5), 0, L.z_el), (1, 0, 0)))
    return b


def fuse_all(shapes):
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    out = out.removeSplitter()
    if len(out.Solids) == 1:
        return out.Solids[0]
    return out


def find_face(solid, normal, pred):
    """Index (1-based) of the planar face whose outward normal matches `normal` and whose centre satisfies `pred`."""
    n = V(*normal)
    best = None
    for i, f in enumerate(solid.Faces):
        if f.Surface.TypeId != "Part::GeomPlane":
            continue
        fn = f.normalAt(0, 0)
        if abs(fn.dot(n)) > 0.99 and pred(f.CenterOfMass, f.Area):          # position picks the face; orientation sign is unreliable
            if best is None or f.Area > best[1]:
                best = (i + 1, f.Area)
    if best is None:
        raise RuntimeError("no face found for %s" % (normal,))
    return "Face%d" % best[0]


def main():
    P = json.load(open(os.path.join(ROOT, "params.json")))
    doc = App.newDocument("PerigeeFEA")
    L, reg = build_defeatured(P)
    print("defeatured parts:", len(reg))
    group_solids = {}
    for g, names in GROUPS.items():
        group_solids[g] = fuse_all([reg[n]["shape"] for n in names if n in reg])
        print("  %s: solids=%d valid=%s" % (g, len(group_solids[g].Solids), group_solids[g].isValid()))
    structure = fuse_all(list(group_solids.values()) + bridges(L))
    print("structure: solids=%d valid=%s faces=%d volume=%.0f cm3" % (len(structure.Solids), structure.isValid(), len(structure.Faces), structure.Volume / 1000))
    if len(structure.Solids) != 1:
        vols = sorted((s.Volume for s in structure.Solids), reverse=True)
        print("WARNING: structure is not one solid, volumes:", [round(v / 1000) for v in vols][:8])

    body = doc.addObject("Part::Feature", "Structure")
    body.Label = "Structure (bonded, defeatured)"
    body.Shape = structure
    for g, s in group_solids.items():
        f = doc.addObject("Part::Feature", "Group_" + g); f.Shape = s; f.Label = "group " + g
        Part.export([f], os.path.join(OUT, "Perigee_FEA_%s.step" % g))
        doc.removeObject(f.Name)
    Part.export([body], os.path.join(OUT, "Perigee_FEA_structure.step"))
    structure.exportBrep(os.path.join(OUT, "Perigee_FEA_structure.brep"))

    # ---- faces for the constraints
    f_foot = find_face(structure, (0, 0, -1), lambda c, a: abs(c.z) < 0.5 and a > 5000)
    f_boom = find_face(structure, (0, 1, 0), lambda c, a: abs(c.y - L.y_boom_end) < 0.5 and abs(c.z - L.z_el) < 1 and a > 1000)
    f_can = find_face(structure, (0, -1, 0), lambda c, a: abs(c.y - L.y_cw_end) < 0.5 and abs(c.z - L.z_el) < 1)   # counterweight arm end ring
    print("faces: foot", f_foot, "boom", f_boom, "canister", f_can)

    # ---- FEM analysis
    import ObjectsFem
    analysis = ObjectsFem.makeAnalysis(doc, "Analysis")
    try:
        solver = ObjectsFem.makeSolverCalculiXCcxTools(doc, "CalculiX")
    except Exception:
        solver = ObjectsFem.makeSolverCalculiX(doc, "CalculiX")
    solver.AnalysisType = "static"
    analysis.addObject(solver)
    mat = ObjectsFem.makeMaterialSolid(doc, "PLA")
    m = dict(mat.Material)
    m.update({"Name": "PLA (printed, 30 % gyroid, 5 walls: stiffness ~60 % of solid)", "YoungsModulus": "2100 MPa", "PoissonRatio": "0.35",
              "Density": "700 kg/m^3", "UltimateTensileStrength": "35 MPa", "YieldStrength": "30 MPa"})
    mat.Material = m
    analysis.addObject(mat)
    fixed = ObjectsFem.makeConstraintFixed(doc, "FixedFoot")
    fixed.References = [(body, f_foot)]
    analysis.addObject(fixed)
    dish_N = P["dish_mass_kg"] * 9.81
    c1 = ObjectsFem.makeConstraintForce(doc, "DishWeight")
    c1.References = [(body, f_boom)]; c1.Force = "%.1f N" % dish_N
    c1.Direction = (body, [f_foot]); c1.Reversed = False
    analysis.addObject(c1)
    c2 = ObjectsFem.makeConstraintForce(doc, "CounterweightFill")
    c2.References = [(body, f_can)]; c2.Force = "62 N"                          # canister + fill, on the arm end
    c2.Direction = (body, [f_foot]); c2.Reversed = False
    analysis.addObject(c2)
    c3 = ObjectsFem.makeConstraintForce(doc, "WindOnDish")
    c3.References = [(body, f_boom)]; c3.Force = "130 N"                      # 1 m dish, ~15 m/s, Cd 1.2, applied at the boom flange
    c3.Direction = (body, [f_boom]); c3.Reversed = False
    analysis.addObject(c3)
    grav = ObjectsFem.makeConstraintSelfWeight(doc, "Gravity")
    analysis.addObject(grav)
    mesh = ObjectsFem.makeMeshGmsh(doc, "Mesh")
    mesh.Shape = body
    mesh.CharacteristicLengthMax = "8 mm"
    mesh.CharacteristicLengthMin = "2 mm"
    mesh.ElementOrder = "2nd"
    analysis.addObject(mesh)
    doc.recompute()
    path = os.path.join(OUT, "Perigee_FEA.FCStd")
    doc.saveAs(path)
    tools_guidoc.inject(path)
    print("wrote", path)
    json.dump({"structure_volume_cm3": round(structure.Volume / 1000), "faces": len(structure.Faces), "solids": len(structure.Solids),
               "fixed_face": f_foot, "dish_face": f_boom, "canister_face": f_can, "dish_N": round(dish_N, 1), "cw_N": 62, "wind_N": 130},
              open(os.path.join(OUT, "fea_setup.json"), "w"), indent=1)


main()
