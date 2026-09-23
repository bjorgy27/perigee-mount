"""Parametric FEA sweep of the Perigee gimbal: dish size x dish mass x wind speed x wind direction x elevation pose.
Run (project venv: gmsh + numpy; CalculiX `ccx` on PATH):

    .venv/bin/python tools_fea_sweep.py mesh          # gmsh: out/fea/sweep/pose_<el>.brep -> pose_<el>.inp (2nd-order tets)
    .venv/bin/python tools_fea_sweep.py solve [el ...] # CalculiX: 9 unit load steps per pose (6 at the dish flange, 2 at the
                                                      #   counterweight arm end, gravity) -> pose_<el>.frd / .dat
    .venv/bin/python tools_fea_sweep.py post          # superpose the unit results over the scenario grid -> SWEEP.md, sweep.csv

Poses come from tools_fea_poses.py (freecadcmd). The structure is linear elastic, so every scenario is an exact linear
combination of the unit steps; nothing is re-solved. Loads for a scenario: dish + feed weight and wind at their centres
(paraboloid shell centroid, feed at the focus), counterweight fill + canister weight at the canister centre, gravity
on the printed structure. Wind: F = 1/2 rho v^2 A Cd(alpha), Cd = 0.3 + 1.0 cos^2(alpha), alpha = wind-to-boresight angle
(solid dish: conservative for a mesh reflector). No dynamic/gust factor."""
import os, sys, json, math, subprocess, time, glob
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(ROOT, "out", "fea", "sweep")
P = json.load(open(os.path.join(ROOT, "params.json")))
REF_DISH, ROT_DISH, REF_CW, ROT_CW = 9000001, 9000002, 9000003, 9000004
UNIT_M = 1000.0                     # N.mm per unit moment step
STEPS = ["dish_Fx", "dish_Fy", "dish_Fz", "dish_Mx", "dish_My", "dish_Mz", "cw_Fz", "cw_Mx", "gravity"]
MAT = dict(E=2100.0, nu=0.35, rho_t_mm3=7.0e-10, yield_MPa=30.0)      # PLA derated for 30 % gyroid, as tools_fea.py
G_MM = 9810.0

# ----------------------------------------------------------------------------------------------- scenario grid
DISH_D = [0.6, 0.8, 1.0, 1.2]                 # m
DISH_M = [3.0, 6.0, 9.0, 12.0, 15.0]          # kg (dish + adapter; feed added separately)
WIND_V = [0.0, 10.0, 15.0, 20.0, 25.0, 30.0, 40.0]   # m/s
WIND_DIR = {"head-on (+Y)": (0.0, 1.0, 0.0), "side (+X)": (1.0, 0.0, 0.0)}
FEED_KG = 0.5
CANISTER_KG = 0.125                           # printed canister + cap (not in the bonded structure)
CW_FILL_MAX_KG = 7.8                          # lead shot, 1.14 L
CW_FILL_OFFSET = -(P["cw_canister_len"] - 4) / 2.0 - 4     # fill centre along the arm axis, from the arm end (mm, outward negative)
CANISTER_OFFSET = -60.0
RHO_AIR = 1.225


def poses():
    return sorted(int(os.path.basename(f)[5:8]) for f in glob.glob(os.path.join(OUT, "pose_???.json")))


def tag(el):
    return "%03d" % el


# ============================================================================================== mesh
def mesh_pose(el, lmin=None, lmax=None, curv=None):
    import gmsh
    lmin = lmin or float(os.environ.get("MESH_LMIN", 2.5)); lmax = lmax or float(os.environ.get("MESH_LMAX", 16)); curv = curv or float(os.environ.get("MESH_CURV", 12))
    meta = json.load(open(os.path.join(OUT, "pose_%s.json" % tag(el))))
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0); gmsh.logger.start()
    gmsh.option.setNumber("General.NumThreads", 16)
    gmsh.model.add("pose")
    gmsh.model.occ.importShapes(os.path.join(OUT, "pose_%s.brep" % tag(el)))
    gmsh.model.occ.synchronize()
    vols = gmsh.model.getEntities(3)
    assert len(vols) == 1, vols
    faces = gmsh.model.getEntities(2)

    def faces_at(center, tol=1.5, min_area=200):
        hits = []
        for d, t in faces:
            c = gmsh.model.occ.getCenterOfMass(2, t)
            if math.dist(c, center) < tol and gmsh.model.occ.getMass(2, t) > min_area:
                hits.append(t)
        return hits

    f_foot = [t for d, t in faces if gmsh.model.getBoundingBox(2, t)[5] < 0.5]
    f_dish = faces_at(meta["dish_center"])
    f_cw = faces_at(meta["cw_center"])
    print("EL %d faces: foot %s dish %s cw %s" % (el, f_foot, f_dish, f_cw))
    assert f_foot and f_dish and f_cw
    gmsh.model.addPhysicalGroup(3, [vols[0][1]], 1, "STRUCT")
    gmsh.option.setNumber("Mesh.MeshSizeMin", lmin)
    gmsh.option.setNumber("Mesh.MeshSizeMax", lmax)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", curv)
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvatureIsotropic", 1)
    gmsh.option.setNumber("Mesh.MinimumCircleNodes", 12)
    gmsh.option.setNumber("Mesh.Algorithm", int(os.environ.get("MESH_ALG2D", 6)))
    small = [t for d, t in gmsh.model.getEntities(1) if gmsh.model.getType(1, t) == "Circle" and
             max(gmsh.model.getBoundingBox(1, t)[3] - gmsh.model.getBoundingBox(1, t)[0], gmsh.model.getBoundingBox(1, t)[4] - gmsh.model.getBoundingBox(1, t)[1]) < 14]
    if small:                                               # small round features (mount holes, spigots): keep their edges fine, grow away from them
        f = gmsh.model.mesh.field.add("Distance"); gmsh.model.mesh.field.setNumbers(f, "CurvesList", small); gmsh.model.mesh.field.setNumber(f, "Sampling", 40)
        th = gmsh.model.mesh.field.add("Threshold"); gmsh.model.mesh.field.setNumber(th, "InField", f)
        gmsh.model.mesh.field.setNumber(th, "SizeMin", lmin); gmsh.model.mesh.field.setNumber(th, "SizeMax", lmax)
        gmsh.model.mesh.field.setNumber(th, "DistMin", 3); gmsh.model.mesh.field.setNumber(th, "DistMax", 30)
        gmsh.model.mesh.field.setAsBackgroundMesh(th)
        gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.Algorithm3D", 10)          # HXT, parallel
    gmsh.option.setNumber("Mesh.ElementOrder", 2)
    gmsh.option.setNumber("Mesh.SecondOrderLinear", 1)      # straight-sided C3D10: curved mid-nodes on fillets inverted elements (ccx: nonpositive jacobian)
    gmsh.option.setNumber("Mesh.Optimize", 1)
    gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)
    gmsh.option.setNumber("Mesh.SaveGroupsOfNodes", 0)
    t0 = time.time()
    try:
        gmsh.model.mesh.generate(3)
    except Exception as e:
        print("  HXT: %s" % str(e).strip()[:80])
    if sum(len(e) for e in gmsh.model.mesh.getElements(3)[1]) == 0:   # HXT rejects some coarse surface meshes; Delaunay is slower but robust
        print("  HXT produced no volume mesh, retrying with Delaunay")
        gmsh.option.setNumber("Mesh.Algorithm3D", 1)
        try:
            gmsh.model.mesh.generate(3)
        except Exception as e:
            print("  Delaunay: %s" % str(e).strip()[:80])
    errs = [m for m in gmsh.logger.get() if m.startswith("Error")]
    if errs:
        print("  gmsh errors: %s" % errs[:3])
    nn = len(gmsh.model.mesh.getNodes()[0])
    ne = sum(len(e) for e in gmsh.model.mesh.getElements(3)[1])
    print("  mesh: %d nodes, %d tets, %.0f s" % (nn, ne, time.time() - t0))
    raw = os.path.join(OUT, "pose_%s_mesh.inp" % tag(el))
    gmsh.write(raw)
    sets = {}
    for name, fl in (("FOOT", f_foot), ("DISH", f_dish), ("CW", f_cw)):
        ids = set()
        for t in fl:
            ids.update(gmsh.model.mesh.getNodes(2, t, True)[0].tolist())
        sets[name] = sorted(int(i) for i in ids)
    gmsh.finalize()
    write_deck(el, raw, sets, meta)
    json.dump({"nodes": nn, "tets": ne, "sets": {k: len(v) for k, v in sets.items()}}, open(os.path.join(OUT, "pose_%s_mesh.json" % tag(el)), "w"))


def write_deck(el, raw, sets, meta):
    txt = open(raw).read()
    # keep only *NODE and the C3D10 *ELEMENT block(s) that gmsh wrote
    lines = []
    keep = False
    for ln in txt.splitlines():
        u = ln.upper()
        if u.startswith("*NODE"):
            keep = True; lines.append("*NODE, NSET=NALL"); continue
        if u.startswith("*ELEMENT"):
            keep = "C3D10" in u
            if keep:
                lines.append("*ELEMENT, TYPE=C3D10, ELSET=STRUCT")
            continue
        if u.startswith("*"):
            keep = False; continue
        if keep:
            lines.append(ln)
    d = meta["dish_center"]; c = meta["cw_center"]
    out = ["*HEADING", "Perigee gimbal unit load cases, EL pose %d deg" % el] + lines
    out.append("*NODE, NSET=REFS")
    out.append("%d, %.4f, %.4f, %.4f" % (REF_DISH, *d)); out.append("%d, %.4f, %.4f, %.4f" % (ROT_DISH, *d))
    out.append("%d, %.4f, %.4f, %.4f" % (REF_CW, *c)); out.append("%d, %.4f, %.4f, %.4f" % (ROT_CW, *c))
    for name, ids in sets.items():
        out.append("*NSET, NSET=%s" % name)
        for i in range(0, len(ids), 12):
            out.append(", ".join(str(x) for x in ids[i:i + 12]))
    out += ["*MATERIAL, NAME=PLA", "*ELASTIC", "%.1f, %.2f" % (MAT["E"], MAT["nu"]), "*DENSITY", "%.3e" % MAT["rho_t_mm3"],
            "*SOLID SECTION, ELSET=STRUCT, MATERIAL=PLA", "*BOUNDARY", "FOOT, 1, 3",
            "*RIGID BODY, NSET=DISH, REF NODE=%d, ROT NODE=%d" % (REF_DISH, ROT_DISH),
            "*RIGID BODY, NSET=CW, REF NODE=%d, ROT NODE=%d" % (REF_CW, ROT_CW)]
    loads = {"dish_Fx": (REF_DISH, 1, 1.0), "dish_Fy": (REF_DISH, 2, 1.0), "dish_Fz": (REF_DISH, 3, 1.0),
             "dish_Mx": (ROT_DISH, 1, UNIT_M), "dish_My": (ROT_DISH, 2, UNIT_M), "dish_Mz": (ROT_DISH, 3, UNIT_M),
             "cw_Fz": (REF_CW, 3, 1.0), "cw_Mx": (ROT_CW, 1, UNIT_M)}
    for s in STEPS:
        out += ["*STEP", "*STATIC, SOLVER=ITERATIVE CHOLESKY"]
        if s == "gravity":
            out += ["*CLOAD, OP=NEW", "*DLOAD, OP=NEW", "STRUCT, GRAV, %.1f, 0., 0., -1." % G_MM]
        else:
            n, dof, val = loads[s]
            out += ["*CLOAD, OP=NEW", "%d, %d, %.3f" % (n, dof, val)]
        out += ["*NODE FILE", "U", "*EL FILE", "S", "*NODE PRINT, NSET=REFS", "U", "*END STEP"]
    open(os.path.join(OUT, "pose_%s.inp" % tag(el)), "w").write("\n".join(out) + "\n")
    os.remove(raw)


# ============================================================================================== solve
def solve_pose(el, threads=16):
    env = dict(os.environ, OMP_NUM_THREADS=str(threads), CCX_NPROC_EQUATION_SOLVER=str(threads))
    log = open(os.path.join(OUT, "pose_%s_ccx.log" % tag(el)), "w")
    t0 = time.time()
    r = subprocess.run(["ccx", "-i", "pose_%s" % tag(el)], cwd=OUT, env=env, stdout=log, stderr=subprocess.STDOUT)
    print("EL %d: ccx exit %d in %.0f s" % (el, r.returncode, time.time() - t0))
    return r.returncode


# ============================================================================================== post
def read_frd(path):
    """-> node ids, xyz, list over steps of (U[n,3], S[n,6]) in frd node order."""
    ids = []; xyz = []; U = []; S = []
    cur = None; blk = None
    with open(path) as f:
        for ln in f:
            if ln.startswith("    2C"):
                blk = "coord"; continue
            if ln.startswith(" -4  DISP"):
                blk = "disp"; cur = []; continue
            if ln.startswith(" -4  STRESS"):
                blk = "stress"; cur = []; continue
            if ln.startswith(" -4") or ln.startswith("  100C") or ln.startswith("    3C"):
                if blk == "disp" and cur: U.append(cur)
                if blk == "stress" and cur: S.append(cur)
                blk = None; cur = None
                continue
            if ln.startswith(" -3"):
                if blk == "disp" and cur: U.append(cur)
                if blk == "stress" and cur: S.append(cur)
                blk = None; cur = None; continue
            if ln.startswith(" -1"):
                if blk == "coord":
                    ids.append(int(ln[3:13])); xyz.append((float(ln[13:25]), float(ln[25:37]), float(ln[37:49])))
                elif blk == "disp":
                    cur.append((float(ln[13:25]), float(ln[25:37]), float(ln[37:49])))
                elif blk == "stress":
                    cur.append(tuple(float(ln[13 + 12 * k:25 + 12 * k]) for k in range(6)))
    ids = np.array(ids); xyz = np.array(xyz)
    U = [np.array(u) for u in U]; S = [np.array(s) for s in S]
    assert len(U) == len(STEPS) and len(S) == len(STEPS), (len(U), len(S))
    return ids, xyz, U, S


def read_dat(path):
    """rotation-node 'displacements' = rotation vector (rad) per step, and ref-node translations."""
    out = []; cur = None
    for ln in open(path):
        if "displacements" in ln:
            cur = {}; out.append(cur); continue
        p = ln.split()
        if cur is not None and len(p) == 4 and p[0].isdigit():
            cur[int(p[0])] = np.array([float(x) for x in p[1:]])
    return out


def von_mises(S):
    sx, sy, sz, txy, tyz, txz = S.T
    return np.sqrt(0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2) + 3 * (txy ** 2 + tyz ** 2 + txz ** 2))


def region_of(xyz, z_el):
    x, y, z = xyz.T
    r_el = np.hypot(y, z - z_el)
    reg = np.full(len(xyz), "pedestal", dtype=object)
    reg[(z > 75)] = "head (puck/drum/plate)"
    reg[(z > 190)] = "columns"
    reg[(np.abs(x) > 46) & (z > z_el - 80)] = "shoulders"
    reg[(np.abs(x) <= 46) & (r_el < 420) & (z > z_el - 420)] = "cradle (hub/boom/arm)"
    return reg


def dish_geometry(D):
    """paraboloid shell with f/D = 0.4: centroid depth from the vertex (mm), focal length (mm), area (m2)."""
    f = 0.4 * D
    r = np.linspace(0, D / 2, 2000); y = r ** 2 / (4 * f); w = 2 * math.pi * r * np.sqrt(1 + (r / (2 * f)) ** 2)
    return float(np.trapezoid(y * w, r) / np.trapezoid(w, r)) * 1000, f * 1000, math.pi * D * D / 4


def rot_x(el):
    a = math.radians(el)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def scenario_coeffs(el, D, m_dish, v, wdir):
    """coefficients on the unit steps for one scenario, plus bookkeeping."""
    R = rot_x(el)
    dc, f, A = dish_geometry(D)
    y0 = 8.0                                                    # adapter plate thickness: dish vertex sits 8 mm past the flange
    r_dish = R @ np.array([0, y0 + dc, 0]); r_feed = R @ np.array([0, y0 + f, 0])
    n_axis = R @ np.array([0, 1.0, 0])
    F = np.zeros(3); M = np.zeros(3)
    for m, r in ((m_dish, r_dish), (FEED_KG, r_feed)):
        fw = np.array([0, 0, -m * 9.81]); F += fw; M += np.cross(r, fw)
    if v > 0:
        w = np.array(wdir, float)
        ca = abs(float(n_axis @ w))
        Cd = 0.3 + 1.0 * ca * ca
        fw = 0.5 * RHO_AIR * v * v * A * Cd * w
        F += fw; M += np.cross(r_dish, fw)
    # counterweight fill sized to balance the dish + feed at EL 0 about the EL axis (lever = arm end offset + fill offset)
    mom_dish = (P["dish_offset"] + y0 + dc) * m_dish + (P["dish_offset"] + y0 + f) * FEED_KG - 34.0   # kg.mm about the EL axis; -34 = printed cradle net (canister side heavier)
    lever_cw = P["hub_size"] / 2 + P["cw_arm_len"] - CW_FILL_OFFSET                                  # fill centre from the EL axis, mm
    m_fill_req = mom_dish / lever_cw
    m_fill = min(max(m_fill_req, 0.0), CW_FILL_MAX_KG)
    r_fill = R @ np.array([0, CW_FILL_OFFSET, 0]); r_can = R @ np.array([0, CANISTER_OFFSET, 0])
    Fc = np.zeros(3); Mc = np.zeros(3)
    for m, r in ((m_fill, r_fill), (CANISTER_KG, r_can)):
        fw = np.array([0, 0, -m * 9.81]); Fc += fw; Mc += np.cross(r, fw)
    c = {"dish_Fx": F[0], "dish_Fy": F[1], "dish_Fz": F[2], "dish_Mx": M[0] / UNIT_M, "dish_My": M[1] / UNIT_M, "dish_Mz": M[2] / UNIT_M,
         "cw_Fz": Fc[2], "cw_Mx": Mc[0] / UNIT_M, "gravity": 1.0}
    info = {"wind_N": float(np.linalg.norm(F - np.array([0, 0, -(m_dish + FEED_KG) * 9.81]))), "cw_fill_kg": m_fill, "cw_fill_req_kg": m_fill_req,
            "el_residual_kgcm": (mom_dish - m_fill * lever_cw) / 10.0}      # unbalanced EL moment left for the Stingray-9 (calm), kg.cm
    return np.array([c[s] for s in STEPS]), info, (M[0] + Mc[0], Fc)


def post():
    rows = []
    verify = None
    data = {}
    for el in poses():
        ids, xyz, U, S = read_frd(os.path.join(OUT, "pose_%s.frd" % tag(el)))
        dat = read_dat(os.path.join(OUT, "pose_%s.dat" % tag(el)))
        meta = json.load(open(os.path.join(OUT, "pose_%s.json" % tag(el))))
        Ust = np.stack(U); Sst = np.stack(S)                      # [step, node, k]
        reg = region_of(xyz, meta["z_el"])
        regions = sorted(set(reg))
        data[el] = (ids, xyz, Ust, Sst, dat, reg, regions, meta)
        print("EL %d: %d nodes, %d steps" % (el, len(ids), len(U)))

    def evaluate(el, coeffs):
        ids, xyz, Ust, Sst, dat, reg, regions, meta = data[el]
        S = np.tensordot(coeffs, Sst, axes=1); Uv = np.tensordot(coeffs, Ust, axes=1)
        vm = von_mises(S); umag = np.linalg.norm(Uv, axis=1)
        rot = sum(c * dat[i][ROT_DISH] for i, c in enumerate(coeffs))
        u_dish = sum(c * dat[i][REF_DISH] for i, c in enumerate(coeffs))
        u_cw = sum(c * dat[i][REF_CW] for i, c in enumerate(coeffs))
        k = int(np.argmax(vm))
        per_reg = {r: float(vm[reg == r].max()) for r in regions}
        return dict(vm_max=float(vm[k]), vm_p999=float(np.percentile(vm, 99.9)), vm_where=reg[k], vm_xyz=xyz[k].round(0).tolist(),
                    u_max=float(umag.max()), u_dish=float(np.linalg.norm(u_dish)), u_cw=float(np.linalg.norm(u_cw)),
                    point_err_deg=float(np.degrees(np.linalg.norm(rot))), per_region=per_reg)

    # ---- verification against the fine-mesh run of 2026-09-21 (RESULTS.md): EL 0, 88 N dish, 130 N +Y wind, 62 N cw, gravity
    if 0 in data:
        c = np.array([0, 130.0, -88.3, 0, 0, 0, -62.0, 0, 1.0])
        verify = evaluate(0, c)

    for el in poses():
        for D in DISH_D:
            for m in DISH_M:
                for name, wdir in WIND_DIR.items():
                    for v in WIND_V:
                        if v == 0 and name != "head-on (+Y)":
                            continue
                        coeffs, info, _ = scenario_coeffs(el, D, m, v, wdir)
                        r = evaluate(el, coeffs)
                        r.update(el=el, D=D, m=m, wind=name if v > 0 else "calm", v=v, **info)
                        r["SF"] = MAT["yield_MPa"] / r["vm_max"]
                        rows.append(r)
    write_report(rows, verify, data)


def write_report(rows, verify, data):
    import csv
    keys = ["el", "D", "m", "wind", "v", "wind_N", "cw_fill_kg", "cw_fill_req_kg", "el_residual_kgcm", "vm_max", "vm_p999", "vm_where", "vm_xyz", "SF", "u_dish", "u_cw", "u_max", "point_err_deg"]
    with open(os.path.join(OUT, "sweep.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(keys + ["vm_" + r for r in sorted(rows[0]["per_region"])])
        for r in rows:
            w.writerow([r[k] if not isinstance(r[k], float) else round(r[k], 4) for k in keys] + [round(r["per_region"][k], 3) for k in sorted(r["per_region"])])
    L = ["# Perigee gimbal FEA sweep (%s)" % time.strftime("%Y-%m-%d"), "",
         "Generated by tools_fea_sweep.py. Linear static superposition of 9 unit load cases per elevation pose (tools_fea_poses.py",
         "geometry: bonded, defeatured printed structure; PLA E = %.0f MPa, yield %.0f MPa; foot fixed; CalculiX 2.23, gmsh)." % (MAT["E"], MAT["yield_MPa"]), "",
         "Meshes: " + "; ".join("EL %d: %s nodes" % (el, "{:,}".format(len(data[el][0]))) for el in sorted(data)), "",
         "Loads per scenario: dish (+ 0.5 kg feed at the focus) weight and wind at the paraboloid centroid, f/D = 0.4, vertex 8 mm past the",
         "boom flange; wind F = 1/2 rho v^2 A Cd, Cd = 0.3 + cos^2(alpha) (1.3 head-on, solid dish); counterweight fill sized to balance",
         "the dish at EL 0 (lead-shot cap %.1f kg), canister + fill weight at the canister centre; gravity on the structure. SF = yield / peak" % CW_FILL_MAX_KG,
         "von Mises. Peak values sit at re-entrant corners of the bonded model and are conservative; the 99.9th percentile is also given.", ""]
    if verify:
        L += ["## Verification against the fine-mesh run (out/fea/RESULTS.md, 372k nodes, same loads at EL 0)", "",
              "| Quantity | Fine mesh 2026-09-21 | This mesh + rigid flange coupling |", "|---|---|---|",
              "| Peak von Mises | 2.68 MPa | %.2f MPa (%s) |" % (verify["vm_max"], verify["vm_where"]),
              "| 99.9th percentile von Mises | 2.09 MPa | %.2f MPa |" % verify["vm_p999"],
              "| Deflection at the boom flange | 2.9 mm | %.2f mm |" % verify["u_dish"],
              "| Deflection at the counterweight arm end | 3.6 mm | %.2f mm |" % verify["u_cw"], ""]
    # summary: worst over wind direction and pose for each (D, m, v)
    L += ["## Worst case over elevation pose and wind direction", "",
          "| Dish | Mass | Wind | Wind force | CW fill | Peak vM | p99.9 vM | SF | Where | Dish deflection | Pointing error |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    by = {}
    for r in rows:
        by.setdefault((r["D"], r["m"], r["v"]), []).append(r)
    for (D, m, v), rs in sorted(by.items()):
        w = max(rs, key=lambda r: r["vm_max"])
        L.append("| %.1f m | %.0f kg | %.0f m/s | %.0f N | %.1f kg%s | %.2f MPa | %.2f | %.1f | %s, EL %d, %s | %.1f mm | %.2f deg |" % (
            D, m, v, w["wind_N"], w["cw_fill_kg"], "" if w["cw_fill_req_kg"] <= CW_FILL_MAX_KG else " (needs %.1f)" % w["cw_fill_req_kg"],
            w["vm_max"], w["vm_p999"], w["SF"], w["vm_where"], w["el"], w["wind"], w["u_dish"], w["point_err_deg"]))
    # wind speed limits per dish/mass
    L += ["", "## Wind speed at which the safety factor drops below 2 (peak von Mises = 15 MPa), interpolated", "",
          "| Dish | " + " | ".join("%.0f kg" % m for m in DISH_M) + " |", "|---|" + "---|" * len(DISH_M)]
    for D in DISH_D:
        cells = []
        for m in DISH_M:
            vs = sorted(set(r["v"] for r in rows if r["D"] == D and r["m"] == m))
            worst = [max(r["vm_max"] for r in rows if r["D"] == D and r["m"] == m and r["v"] == v) for v in vs]
            lim = None
            for i in range(1, len(vs)):
                if worst[i] >= 15.0 and worst[i - 1] < 15.0:
                    lim = vs[i - 1] + (15.0 - worst[i - 1]) / (worst[i] - worst[i - 1]) * (vs[i] - vs[i - 1]); break
            cells.append("> %.0f m/s" % vs[-1] if lim is None and worst[-1] < 15 else ("%.0f m/s" % lim if lim else "static fail"))
        L.append("| %.1f m | " % D + " | ".join(cells) + " |")
    L += ["", "## Counterweight and elevation servo (calm, EL 0)", "",
          "Fill needed to balance the dish + feed about the EL axis, capped at the canister's lead-shot capacity; the unbalanced remainder",
          "is what the Stingray-9 (227 kg.cm stall) must hold. The FEA above always uses the capped fill.", "",
          "| Dish | Mass | Fill needed | Fill used | Unbalanced EL moment | Share of 227 kg.cm |", "|---|---|---|---|---|---|"]
    for r in rows:
        if r["el"] == 0 and r["v"] == 0:
            L.append("| %.1f m | %.0f kg | %.1f kg | %.1f kg | %.0f kg.cm | %.0f %% |" % (r["D"], r["m"], r["cw_fill_req_kg"], r["cw_fill_kg"], r["el_residual_kgcm"], 100 * r["el_residual_kgcm"] / 227))
    L += ["", "## Where the stress goes (1.0 m, 9 kg, 20 m/s)", "",
          "| Pose | Wind | " + " | ".join(sorted(rows[0]["per_region"])) + " |", "|---|---|" + "---|" * len(rows[0]["per_region"])]
    for r in rows:
        if r["D"] == 1.0 and r["m"] == 9.0 and r["v"] == 20.0:
            L.append("| EL %d | %s | " % (r["el"], r["wind"]) + " | ".join("%.2f" % r["per_region"][k] for k in sorted(r["per_region"])) + " |")
    L += ["", "Full grid: sweep.csv (%d scenarios)." % len(rows)]
    open(os.path.join(OUT, "SWEEP.md"), "w").write("\n".join(L) + "\n")
    print("wrote SWEEP.md, sweep.csv (%d rows)" % len(rows))


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "post"
    sel = [int(a) for a in sys.argv[2:]] or poses()
    if cmd == "mesh":
        for el in sel:
            mesh_pose(el)
    elif cmd == "solve":
        for el in sel:
            solve_pose(el)
    elif cmd == "post":
        post()
