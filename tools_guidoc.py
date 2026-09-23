"""Give a headless-built FCStd the GuiDocument.xml that FreeCAD's GUI needs.

A file saved by freecadcmd has no GuiDocument.xml. FreeCAD 1.x then restores every view provider with
Visibility = false, so the model opens as an empty 3D view. This writes a minimal GuiDocument.xml into the zip:
one ViewProvider entry per object with its Visibility (taken from Document.xml), the filament colour and
transparency where the object carries a `Filament` property, and expanded tree nodes for the assembly groups.

Usage:  python3 tools_guidoc.py out/PerigeeGimbal.FCStd [more.FCStd ...]     (pure Python, no FreeCAD needed)
build.py calls inject() on the master file and on every per-part file it writes.
"""
import os, re, shutil, sys, tempfile, zipfile
from xml.sax.saxutils import escape

# filament -> (r, g, b, transparency %) ; must match parts.COLORS / PerigeeColors.FCMacro
FILAMENT_RGBA = {
    "black": (0.09, 0.09, 0.10, 0), "orange": (1.00, 0.45, 0.05, 0), "white": (0.93, 0.93, 0.92, 0),
    "grey": (0.42, 0.44, 0.47, 0), "clear": (0.80, 0.90, 1.00, 55), "ref": (0.55, 0.60, 0.65, 75),
    "steel": (0.75, 0.76, 0.78, 0), "brass": (0.80, 0.62, 0.25, 0),
}
EXPANDED_TYPES = {"Assembly::AssemblyObject", "App::Part"}
# origin planes / axes / points carry Visibility=true in Document.xml but a GUI-made file keeps them hidden
HIDDEN_TYPES = {"App::Origin", "App::Plane", "App::Line", "App::Point"}


def packed_color(r, g, b):
    """App::PropertyColor stores 0xRRGGBBAA as an unsigned int (alpha byte is 0 for opaque in FreeCAD's convention)."""
    return (int(round(r * 255)) << 24) | (int(round(g * 255)) << 16) | (int(round(b * 255)) << 8)


def scan_document_xml(xml):
    """Return [(name, type, visible, filament)] for every object in Document.xml."""
    objs = []
    types = {m.group(2): m.group(1) for m in re.finditer(r'<Object type="([^"]+)" name="([^"]+)"', xml)}
    for m in re.finditer(r'<Object name="([^"]+)"[^>]*>(.*?)</Object>', xml, re.S):
        name, body = m.group(1), m.group(2)
        vis = re.search(r'name="Visibility"[^>]*>\s*<Bool value="(\w+)"', body)
        fil = re.search(r'name="Filament"[^>]*>\s*<String value="([^"]*)"', body)
        typ = types.get(name, "")
        visible = (vis.group(1) == "true") if vis else True
        if typ in HIDDEN_TYPES:
            visible = False
        objs.append((name, typ, visible, fil.group(1) if fil else None))
    return objs


def gui_document_xml(objs, colors=True):
    vps = []
    for name, typ, visible, filament in objs:
        props = ['<Property name="Visibility" type="App::PropertyBool"><Bool value="%s"/></Property>' % ("true" if visible else "false")]
        if colors and filament in FILAMENT_RGBA and typ == "Part::Feature":
            r, g, b, tr = FILAMENT_RGBA[filament]
            props.append('<Property name="ShapeColor" type="App::PropertyColor"><PropertyColor value="%d"/></Property>' % packed_color(r, g, b))
            if tr:
                props.append('<Property name="Transparency" type="App::PropertyPercent"><Integer value="%d"/></Property>' % tr)
        vps.append('<ViewProvider name="%s" expanded="%d"><Properties Count="%d">%s</Properties></ViewProvider>' % (
            escape(name), 1 if typ in EXPANDED_TYPES else 0, len(props), "".join(props)))
    return ("<?xml version='1.0' encoding='utf-8'?>\n<Document SchemaVersion=\"1\">\n<ViewProviderData Count=\"%d\">\n%s\n</ViewProviderData>\n</Document>\n"
            % (len(vps), "\n".join(vps)))


def inject(path, colors=True):
    """Rewrite the FCStd at `path` with a GuiDocument.xml (replacing any existing one). Returns the object count."""
    with zipfile.ZipFile(path) as zin:
        xml = zin.read("Document.xml").decode("utf-8", "replace")
        objs = scan_document_xml(xml)
        gui = gui_document_xml(objs, colors)
        fd, tmp = tempfile.mkstemp(suffix=".FCStd", dir=os.path.dirname(os.path.abspath(path)))
        os.close(fd)
        with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as zout:
            for it in zin.infolist():
                if it.filename == "GuiDocument.xml":
                    continue
                zout.writestr(it, zin.read(it.filename))
            zout.writestr("GuiDocument.xml", gui)
    shutil.move(tmp, path)
    os.chmod(path, 0o644)                                  # mkstemp creates 0600
    return len(objs)


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(p, inject(p), "view providers")
