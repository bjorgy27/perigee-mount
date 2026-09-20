"""OpenGL (moderngl, headless EGL) renderer for the Perigee gimbal: stills, spot sheets and the assembly film.
Run with the project venv:  .venv/bin/python tools_render.py [--anim] [--spot t1,t2,...]"""
import json, struct, math, sys, os
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import moderngl

ROOT = os.path.dirname(os.path.abspath(__file__))
meta = json.load(open(os.path.join(ROOT, "viewer", "meta.json")))
COL = {k: tuple(v) for k, v in meta["colors"].items()}
COL.setdefault("brass", (0.80, 0.62, 0.25, 0.0))
FPS = 30
SIZE = (1600, 1200)
BG = (0.075, 0.09, 0.115)


def load_glb(path):
    b = open(path, "rb").read()
    jl = struct.unpack_from("<I", b, 12)[0]
    js = json.loads(b[20:20 + jl]); off = 20 + jl + 8
    bv, acc = js["bufferViews"], js["accessors"]
    def view(i, dtype, n):
        v = bv[acc[i]["bufferView"]]
        return np.frombuffer(b, dtype=dtype, count=acc[i]["count"] * n, offset=off + v["byteOffset"]).reshape(-1, n)
    return view(0, "<f4", 3).astype(np.float32), view(1, "<u4", 1).reshape(-1, 3)


def flat_mesh(pos, idx):
    tri = pos[idx]                                   # (n,3,3)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= (np.linalg.norm(n, axis=1)[:, None] + 1e-12)
    nrm = np.repeat(n[:, None, :], 3, axis=1)
    return np.concatenate([tri.reshape(-1, 3), nrm.reshape(-1, 3)], axis=1).astype(np.float32)


VERT = """
#version 330
uniform mat4 mvp; uniform mat4 model;
in vec3 in_pos; in vec3 in_nrm;
out vec3 v_nrm; out vec3 v_pos;
void main(){ vec4 wp = model * vec4(in_pos,1.0); v_pos = wp.xyz; v_nrm = mat3(model) * in_nrm; gl_Position = mvp * vec4(in_pos,1.0); }
"""
FRAG = """
#version 330
uniform vec4 color; uniform vec3 eye; uniform vec3 l1; uniform vec3 l2;
in vec3 v_nrm; in vec3 v_pos; out vec4 f;
void main(){
  vec3 n = normalize(v_nrm); vec3 vdir = normalize(eye - v_pos);
  if (dot(n, vdir) < 0.0) n = -n;
  float d1 = max(dot(n, normalize(l1)), 0.0), d2 = max(dot(n, normalize(l2)), 0.0);
  vec3 h1 = normalize(normalize(l1) + vdir); float sp = pow(max(dot(n, h1), 0.0), 48.0);
  float rim = pow(1.0 - max(dot(n, vdir), 0.0), 3.0) * 0.18;
  vec3 c = color.rgb * (0.30 + 0.62 * d1 + 0.22 * d2) + vec3(0.35) * sp * (0.25 + 0.75 * step(0.5, color.r + color.g)) + rim;
  c = pow(clamp(c, 0.0, 1.0), vec3(1.0/1.6));
  f = vec4(c, color.a);
}
"""
GRID_VERT = """
#version 330
uniform mat4 mvp; in vec3 in_pos; out vec3 v_pos;
void main(){ v_pos = in_pos; gl_Position = mvp * vec4(in_pos,1.0); }
"""
GRID_FRAG = """
#version 330
in vec3 v_pos; out vec4 f; uniform vec3 bg;
void main(){
  vec2 g = abs(fract(v_pos.xy / 100.0 - 0.5) - 0.5) / fwidth(v_pos.xy / 100.0);
  float line = 1.0 - min(min(g.x, g.y), 1.0);
  float r = length(v_pos.xy); float fade = exp(-r / 1400.0);
  vec3 c = mix(bg, bg + vec3(0.10, 0.12, 0.14), line * fade);
  f = vec4(c, 1.0);
}
"""


class Renderer:
    def __init__(self, size=SIZE, samples=8):
        self.ctx = moderngl.create_standalone_context(backend="egl")
        self.size = size
        self.prog = self.ctx.program(vertex_shader=VERT, fragment_shader=FRAG)
        self.gprog = self.ctx.program(vertex_shader=GRID_VERT, fragment_shader=GRID_FRAG)
        w, h = size
        self.fbo_ms = self.ctx.framebuffer(color_attachments=[self.ctx.renderbuffer((w, h), samples=samples)], depth_attachment=self.ctx.depth_renderbuffer((w, h), samples=samples))
        self.fbo = self.ctx.framebuffer(color_attachments=[self.ctx.renderbuffer((w, h))])
        self.vaos = {}
        for p in meta["parts"]:
            self.vaos[p["name"]] = self._vao(*load_glb(os.path.join(ROOT, "viewer", p["file"])))
        self.hw_vaos = {k: self._vao(*load_glb(os.path.join(ROOT, "viewer", v["file"]))) for k, v in meta.get("hardware_kinds", {}).items()}
        gsz = 3000.0
        quad = np.array([[-gsz, -gsz, 0], [gsz, -gsz, 0], [gsz, gsz, 0], [-gsz, -gsz, 0], [gsz, gsz, 0], [-gsz, gsz, 0]], dtype=np.float32)
        self.grid = self.ctx.simple_vertex_array(self.gprog, self.ctx.buffer(quad.tobytes()), "in_pos")
        self.font = None
        for fp in ("/usr/share/fonts/noto/NotoSansMono-Bold.ttf", "/usr/share/fonts/TTF/DejaVuSansMono-Bold.ttf"):
            if os.path.exists(fp):
                self.font = ImageFont.truetype(fp, 22); break

    def _vao(self, pos, idx):
        data = flat_mesh(pos, idx)
        vbo = self.ctx.buffer(data.tobytes())
        vao = self.ctx.vertex_array(self.prog, [(vbo, "3f 3f", "in_pos", "in_nrm")])
        c = pos.mean(axis=0)
        return vao, c

    # ---- math helpers
    @staticmethod
    def look_at(eye, target, up=(0, 0, 1)):
        eye, target, up = map(lambda a: np.array(a, np.float64), (eye, target, up))
        f = target - eye; f /= np.linalg.norm(f)
        s = np.cross(f, up); s /= np.linalg.norm(s); u = np.cross(s, f)
        M = np.eye(4); M[0, :3] = s; M[1, :3] = u; M[2, :3] = -f
        M[:3, 3] = -M[:3, :3] @ eye
        return M

    @staticmethod
    def perspective(fov_deg, aspect, near, far):
        f = 1.0 / math.tan(math.radians(fov_deg) / 2)
        M = np.zeros((4, 4)); M[0, 0] = f / aspect; M[1, 1] = f; M[2, 2] = (far + near) / (near - far); M[2, 3] = 2 * far * near / (near - far); M[3, 2] = -1
        return M

    def render(self, out, az=0.0, el=0.0, view_dir=(1, -1.3, 0.8), only=None, skip=None, zoom=1.0, center=None, extent=None, anim_t=None, title="", fov=38.0):
        kin = meta["kinematics"]
        A = meta.get("assembly")
        part_anim, hw_anim = {}, {}
        if anim_t is not None and A:
            for it in A["items"]:
                pr = (anim_t - it["t0"]) / it["dur"]
                if it["kind"] == "part":
                    part_anim[it["name"]] = (pr, np.array(it["approach"]) * it["dist"] * (1 - ease(pr)))
                else:
                    hw_anim[it["index"]] = (pr, it["dist"] * (1 - ease(pr)))
        # camera
        d = np.array(view_dir, np.float64); d /= np.linalg.norm(d)
        c = np.array(center if center is not None else (0, 0, 420), np.float64)
        ext = (extent if extent is not None else 900.0) / zoom
        dist = ext / math.tan(math.radians(fov) / 2) * 1.05
        eye = c + d * dist
        V_ = self.look_at(eye, c)
        P_ = self.perspective(fov, self.size[0] / self.size[1], max(1.0, dist * 0.05), dist * 4 + 3000)
        VP = P_ @ V_
        ctx = self.ctx
        self.fbo_ms.use(); ctx.clear(*BG, 1.0); ctx.enable(moderngl.DEPTH_TEST); ctx.disable(moderngl.CULL_FACE)
        self.gprog["mvp"].write(VP.T.astype(np.float32).tobytes()); self.gprog["bg"].value = BG
        self.grid.render()
        self.prog["eye"].value = tuple(eye.astype(np.float32)); self.prog["l1"].value = (0.5, -0.7, 0.9); self.prog["l2"].value = (-0.8, 0.5, 0.3)
        draws = []
        for p in meta["parts"]:
            if p["hidden"] or (only and p["group"] not in only and p["name"] not in only) or (skip and (p["name"] in skip or p["group"] in skip)):
                continue
            off = np.zeros(3)
            if anim_t is not None:
                pr, off = part_anim.get(p["name"], (1.0, np.zeros(3)))
                if pr < 0: continue
            R, t = pose_matrix(p["group"], az, el, kin)
            M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t + off
            vao, cen = self.vaos[p["name"]]
            draws.append((vao, M, COL[p["color"]], (M[:3, :3] @ cen + M[:3, 3])))
        if not (skip and "hardware" in skip):
            for hi, inst in enumerate(meta.get("hardware", [])):
                if only and inst["group"] not in only: continue
                if skip and inst["group"] in skip: continue
                M4 = np.array(inst["matrix"], np.float64)
                if anim_t is not None:
                    pr, dd = hw_anim.get(hi, (1.0, 0.0))
                    if pr < 0: continue
                    M4 = M4.copy(); M4[:3, 3] += M4[:3, 2] * dd
                R, t = pose_matrix(inst["group"], az, el, kin)
                G4 = np.eye(4); G4[:3, :3] = R; G4[:3, 3] = t
                M = G4 @ M4
                vao, cen = self.hw_vaos[inst["kind"]]
                draws.append((vao, M, COL[meta["hardware_kinds"][inst["kind"]]["color"]], M[:3, 3]))
        opaque = [dw for dw in draws if dw[2][3] == 0]
        trans = sorted([dw for dw in draws if dw[2][3] > 0], key=lambda dw: -np.linalg.norm(dw[3] - eye))
        ctx.depth_mask = True; ctx.disable(moderngl.BLEND)
        for vao, M, col, _ in opaque:
            self._draw(vao, M, VP, col)
        ctx.enable(moderngl.BLEND); ctx.blend_func = moderngl.SRC_ALPHA, moderngl.ONE_MINUS_SRC_ALPHA; ctx.depth_mask = False
        for vao, M, col, _ in trans:
            self._draw(vao, M, VP, col)
        ctx.depth_mask = True
        ctx.copy_framebuffer(self.fbo, self.fbo_ms)
        img = Image.frombytes("RGB", self.size, self.fbo.read(components=3)).transpose(Image.FLIP_TOP_BOTTOM)
        if title:
            dr = ImageDraw.Draw(img); dr.text((18, 14), title, fill=(225, 230, 236), font=self.font)
        img.save(out)
        return img

    def _draw(self, vao, M, VP, col):
        self.prog["model"].write(M.T.astype(np.float32).tobytes())
        self.prog["mvp"].write((VP @ M).T.astype(np.float32).tobytes())
        self.prog["color"].value = (col[0], col[1], col[2], 1.0 - col[3])
        vao.render(moderngl.TRIANGLES)


def ease(p):
    p = min(1.0, max(0.0, p)); return 1 - (1 - p) ** 3


def smooth(p):
    p = min(1.0, max(0.0, p)); return p * p * (3 - 2 * p)


def rot(axis, deg):
    a = np.array(axis, float); a /= np.linalg.norm(a); t = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


def pose_matrix(group, az, el, kin):
    Rz = rot([0, 0, 1], az); oz = np.array(kin["az_axis"]["origin"])
    def about(R, o):
        return R, o - R @ o
    if group == "Base":
        return np.eye(3), np.zeros(3)
    Rh, th = about(Rz, oz)
    if group == "Head":
        return Rh, th
    if group == "Cradle":
        Re, te = about(rot([1, 0, 0], el), np.array(kin["el_axis"]["origin"]))
        return Rh @ Re, Rh @ te + th
    g = kin["gears"][group]
    ang = el * g["ratio_vs_el"] if group == "ElPinion" else az * g["ratio_vs_az"]
    Rg, tg = about(rot(g["axis"], ang), np.array(g["origin"]))
    return Rh @ Rg, Rh @ tg + th


def shot_camera(A, t):
    shots = A["shots"]; blend = A.get("blend", 0.7)
    def cam_in(sh, tt):
        f = min(1.0, max(0.0, (tt - sh["t0"]) / max(1e-6, sh["t1"] - sh["t0"])))
        return (np.array(sh["center"], float), sh["extent"] * (1 - 0.07 * f), sh["theta"] + sh["sweep"] * f, sh["elev"])
    k = 0
    for i, sh in enumerate(shots):
        if t >= sh["t0"]: k = i
    cur = shots[k]
    c, e, th, el = cam_in(cur, t)
    if k > 0 and t - cur["t0"] < blend:
        pc, pe, pth, pel = cam_in(shots[k - 1], shots[k - 1]["t1"])
        p = smooth((t - cur["t0"]) / blend)
        dd = ((th - pth + 180) % 360) - 180
        c = pc + (c - pc) * p; e = pe + (e - pe) * p; th = pth + dd * p; el = pel + (el - pel) * p
    return c, e, th, el, cur["label"]


_R = None
def R():
    global _R
    if _R is None: _R = Renderer()
    return _R


def anim_frame(i, od):
    A = meta["assembly"]; t = i / FPS
    c, ext, th, el, label = shot_camera(A, t)
    thr, elr = math.radians(th), math.radians(el)
    vd = (math.cos(thr) * math.cos(elr), math.sin(thr) * math.cos(elr), math.sin(elr))
    done = sum(1 for it in A["items"] if it["kind"] == "part" and t >= it["t0"] + it["dur"])
    R().render(os.path.join(od, "f_%04d.png" % i), 0, 0, view_dir=vd, center=tuple(c), extent=ext, anim_t=t,
               title="PERIGEE gimbal assembly   T+%05.1fs   %s   (%d parts placed)" % (t, label, done))


def render_animation():
    import subprocess
    od = os.path.join(ROOT, "out", "renders", "anim"); os.makedirs(od, exist_ok=True)
    for f in os.listdir(od): os.remove(os.path.join(od, f))
    n = int(meta["assembly"]["total"] * FPS) + FPS
    for i in range(n):
        anim_frame(i, od)
        if i % 200 == 0: print("frame %d/%d" % (i, n), flush=True)
    mp4 = os.path.join(ROOT, "out", "renders", "assembly.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", os.path.join(od, "f_%04d.png"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "19", "-preset", "slow", mp4], check=True)
    print("wrote", mp4, "(%d frames)" % n)


def contact_sheet(times):
    od = os.path.join(ROOT, "out", "renders", "anim"); os.makedirs(od, exist_ok=True)
    idx = [int(t * FPS) for t in times]
    for i in idx: anim_frame(i, od)
    tiles = [Image.open(os.path.join(od, "f_%04d.png" % i)).resize((640, 480)) for i in idx]
    cols = 3; rows = (len(tiles) + cols - 1) // cols
    sheet = Image.new("RGB", (640 * cols, 480 * rows), (0, 0, 0))
    for k, tile in enumerate(tiles): sheet.paste(tile, ((k % cols) * 640, (k // cols) * 480))
    out = os.path.join(ROOT, "out", "renders", "contact_sheet.png"); sheet.save(out); print("wrote", out)


if __name__ == "__main__":
    if "--anim" in sys.argv:
        render_animation(); sys.exit(0)
    if "--spot" in sys.argv:
        contact_sheet([float(x) for x in sys.argv[sys.argv.index("--spot") + 1].split(",")]); sys.exit(0)
    od = os.path.join(ROOT, "out", "renders"); os.makedirs(od, exist_ok=True)
    r = R()
    r.render(os.path.join(od, "iso_front.png"), 0, 0, title="iso front-right")
    r.render(os.path.join(od, "iso_back.png"), 0, 0, view_dir=(-1, 1.3, 0.7), title="iso back-left")
    r.render(os.path.join(od, "el90.png"), 30, 90, view_dir=(1, -1.3, 0.5), title="counterweight down")
    r.render(os.path.join(od, "no_dish_front.png"), 0, 0, skip=["Dish_Reference_1m", "Feed_Reference"], view_dir=(1, -1.2, 0.6), center=(0, 0, 380), extent=620, title="structure")
    r.render(os.path.join(od, "no_dish_left.png"), 0, 0, skip=["Dish_Reference_1m", "Feed_Reference"], view_dir=(-1, -1.0, 0.5), center=(0, 0, 380), extent=620, title="structure, left")
    r.render(os.path.join(od, "base_internals.png"), 0, 0, only=["Base", "AzPinion", "Idler", "Sensor"], view_dir=(0.6, -1, 1.2), center=(0, 0, 30), extent=150, title="base internals (head removed)")
    r.render(os.path.join(od, "el_drive.png"), 0, 30, only=["Head", "Cradle", "ElPinion"], skip=["Dish_Reference_1m", "Feed_Reference", "Electronics_Bay", "Bay_Lid", "Counterweight_Canister", "Counterweight_Cap", "Counterweight_Arm", "Dish_Boom", "Dish_Adapter_Plate"], view_dir=(1, -0.6, 0.35), center=(70, 0, 520), extent=170, title="EL drive close-up")
    r.render(os.path.join(od, "el_drive_inboard.png"), 0, 30, only=["Cradle", "ElPinion", "Head"], skip=["Dish_Reference_1m", "Feed_Reference", "Arm_R_3", "Arm_R_2", "El_Servo_Housing", "El_Housing_Backplate", "El_Servo_placeholder", "Electronics_Bay", "Bay_Lid", "Counterweight_Canister", "Counterweight_Cap"], view_dir=(1, -0.8, 0.3), center=(40, 0, 540), extent=170, title="EL gear + pinion inboard (right arm hidden)")
    r.render(os.path.join(od, "arm_closeup.png"), 0, 0, only=["Head"], skip=["Electronics_Bay", "Bay_Lid", "Logo_Light_Pipe"], view_dir=(1, -0.35, 0.25), center=(90, 0, 250), extent=230, title="right arm")
    r.render(os.path.join(od, "arm_left.png"), 0, 0, only=["Head"], skip=["Electronics_Bay", "Bay_Lid", "Logo_Light_Pipe"], view_dir=(-1, -0.35, 0.25), center=(-90, 0, 250), extent=230, title="left arm")
    print("stills done")
