"""Bundle viewer/parts/*.glb into viewer/parts_bundle.json (base64) so the viewer can be published where only text/JSON is served."""
import base64, json, os
ROOT = os.path.dirname(os.path.abspath(__file__))
pd = os.path.join(ROOT, "viewer", "parts")
bundle = {f[:-4]: base64.b64encode(open(os.path.join(pd, f), "rb").read()).decode() for f in sorted(os.listdir(pd)) if f.endswith(".glb")}
out = os.path.join(ROOT, "viewer", "parts_bundle.json")
json.dump(bundle, open(out, "w"), separators=(",", ":"))
print("bundled", len(bundle), "parts ->", out, round(os.path.getsize(out) / 1e6, 2), "MB")
