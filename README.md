# PERIGEE gimbal

Fully 3D-printed azimuth/elevation pedestal for a dish up to 1 m (mesh), 15-20 lb payload, driven directly by two
goBILDA Stingray servo gearboxes (Stingray-4 on azimuth, 450 deg of travel; Stingray-9 on elevation, 200 deg), with an
AS5600 magnetic encoder on the elevation axis and the Stingrays' own feedback wires for position, controlled by an
Arduino Uno on the rotating head (Bluetooth serial to the PC running Perigee). Azimuth needs no slip ring: 1.25 turns
of travel run on a cable service loop. The azimuth axis runs on a greased printed plain bearing; elevation runs on the
Stingray-9's own ball-bearing hub-shaft on the right and a printed split bushing on the left. The only non-printed parts
are metric screws and nuts, the two gearboxes and the electronics.

Everything here is generated from `params.json` by `build.py` running inside FreeCAD 1.1 (headless).

```
./build.sh                              # regenerate everything (about 30 s): parts, assembly, exploded view, audit, exports, viewer bundle
.venv/bin/python tools_render.py        # OpenGL inspection renders into out/renders (moderngl on headless EGL, ~1 s)
.venv/bin/python tools_render.py --anim # cinematic assembly film -> out/renders/assembly.mp4 (30 fps, ~2 min)
.venv/bin/python tools_render.py --spot 1.5,9.5,36   # contact sheet of film frames at those seconds
python3 -m venv .venv && .venv/bin/pip install moderngl numpy pillow   # one-time renderer setup
```

The build also runs a **geometry audit** (every part one valid closed solid, BOP-checked, watertight mesh:
`out/geometry_audit.txt`) and an **overlap audit** over every pair of parts and every fastener against every part
(`out/overlap_audit.txt`); both must say `clean`. Export folders are wiped and regenerated on every build, so no
stale parts from older revisions linger. It also creates an **Exploded_Assembly** view (Assembly workbench > Exploded Views) with one move per part in assembly order,
so FreeCAD itself can animate the explosion with its slider.

## Hardware is real, and nothing threads into plastic
Every screw and nut is modelled (about 170 pieces) as App::Link instances of master parts in
`Hardware_Library`, placed with the same coordinates that cut their counterbores, nut pockets and nut traps.
All fasteners are ISO 4762 socket head cap screws in M3 / M4 / M5 with ISO 4032 hex nuts. No heat-set inserts and
no self-tapping: each bolt ends in a nut, or in one of the gearboxes' own M4 threads, that sits either in a blind hex pocket on the far face (where that face is
reachable at assembly time) or in a side-loaded nut trap (a slot the nut slides into before the mating part goes on:
pedestal wall, pillar feet, arm feet, shoulder tops, around the left bore, servo window, behind the gearbox block, hub rim). The hub is
a hollow box with a bolted lid so all its flange nuts sit inside. The BOM lists exact counts per size and length.
Tools: 2.5 / 3 / 4 mm hex keys and 5.5 / 7 / 8 mm spanners.

## Styling
Canadarm / R.O.B. proportions: black pedestal with a wide flange plate, white azimuth housing under a plain round
yoke plate, two slender 60 mm round columns rising to 150 mm cylindrical shoulder drums on the elevation axis, a
rounded black hub between them, round black boom and counterweight tubes, opaque black counterweight canister with an
orange cap. Translucent parts show the mechanism: the lens on the right drum over the Stingray-9, the access cover on
the yoke plate, the status window on the left cap. Cables run inside the hollow columns.
Filaments: white (housing, plate, columns, drums, bushings), black (pedestal, flange plate, hub, lid, stub, boom,
counterweight arm and canister), grey (left axis cap, adapter), orange (counterweight cap), translucent (lens,
access cover, window, light pipe).

## What you get

| Path | Contents |
|---|---|
| `out/PerigeeGimbal.FCStd` | Master file: Spreadsheet `Params`, Assembly `Gimbal` with 3 rigid groups, 2 joints (AZ + EL revolutes) and a `Track_Pass_Simulation` |
| `out/parts_fcstd/*.FCStd` | One FreeCAD file per printed part, already in print orientation |
| `out/step/*.step` | STEP per printed part (world coordinates) |
| `out/stl/*.stl` | STL per printed part, oriented for the Bambu A1 bed (largest is 236 mm) |
| `out/print/` | Print bundle: a folder per filament colour with those STLs, plus one 3MF per colour holding all of its parts for a Bambu Studio plate |
| `out/BOM.md` | Printed parts with filament colour and mass estimate, hardware list, key numbers |
| `out/manifest.json` | Machine-readable part list (group, colour, print bbox, volume) |
| `viewer/` | Interactive three.js viewer (`index.html` + `meta.json` + `parts_bundle.json`); pass simulation, fastener toggle, and an **Assemble** mode that builds the gimbal from the base up with a scrub bar |
| `out/renders/assembly.mp4` | Rendered assembly animation: 18 shots, each sub-assembly framed close from its best angle, 360 spins after the yoke and at the end, every fastener arriving along its own axis. Shot list = `SHOTS` in `build.py`, shared with the viewer |
| `macros/PerigeeColors.FCMacro` | Run once in the FreeCAD GUI: attaches the Assembly view providers (exploded view, joints, simulation), applies filament colours, hides joint markers |
| `macros/PerigeeAnimate.FCMacro` | Run in the GUI: animates an overhead pass with flip-over EL (no solver needed) |

### Animation inside FreeCAD
Open the master file, run `PerigeeColors.FCMacro`, then either run `PerigeeAnimate.FCMacro`, or open
Assembly workbench > Simulation > `Track_Pass_Simulation`, press *Generate* then *Play*. The simulation drives the AZ
and EL joints with formulas (`18*time` and a harmonic 0-180 deg sweep).

## Design summary (v8: Stingray direct drives, round columns and drums)

- **Frame**: Z up, AZ axis = Z, EL axis = X at z = 599 mm (510 mm above the deck). Boresight at EL 0 is +Y.
- **Pedestal**: black 216 mm pedestal body on a 236 mm foot flange (6x M6 for a mount plate), 75 mm tall; a black
  236 mm flange plate on top is the AZ retainer (12x M4x20) and the visible rotation seam. Everything fits a 256 mm
  bed with room for a brim.
- **AZ bearing**: printed plain bearing. The puck's 15 mm lip (OD 191) is the journal in the 192 mm bore; the step at
  the puck underside (r 84-96) is the greased thrust face; the flange plate traps the lip with 0.8 mm play. The
  Stingray-4's own bearings centre the shaft; the plain bearing carries the weight and moments.
- **AZ drive**: goBILDA Stingray-4 (3215-0001-0004, feedback mode) stands on four pillars on the pedestal floor,
  bolted from below (4x M4x40 into its 16 mm grid). Its output gear faces up and its four M4 standoffs bolt to the
  underside of the puck (4x M4x25 from the deck). 450 deg of travel, 100 kg.cm, 15 rpm. Position comes from the
  gearbox's feedback wire. Cables pass through a hole in the puck into the pedestal annulus as a service loop.
- **AZ housing**: the rotating electronics bay is a white drum of the same 216 mm diameter as the pedestal body; four
  internal pillars tied to the wall by ribs carry the yoke (M4 top and bottom, nuts trapped in the pillars). Its lid is
  an oval 244 x 220 mm yoke plate with a translucent round access cover; the columns bolt to the plate from below and
  their cables pass up through it.
- **Yoke**: two hollow 60 mm round columns (4 mm walls), each printed as a lower segment and an upper segment joined
  by a solid spigot with 2x M4x50 across; the upper segment carries the lower half of a 150 mm cylindrical shoulder
  drum on the elevation axis. The drums are split at the axis; their upper halves are the bolted caps (2x M4x25 each).
  Feet: 4x M4x25 up from under the yoke plate into side-loaded nuts in a solid foot boss.
- **EL drive**: goBILDA Stingray-9 (3215-0001-0009, feedback mode) sits inside the right shoulder drum, entered
  from the inner face, servo below the axis. The block is held by 8x M4 (4x M4x50 and 4x M4x20) from the drum surface
  into its side M4 threads. Its 88 mm output gear turns in the gap between drum and hub and its four M4 standoffs bolt
  straight to the hub's right wall (4x M4x12 from inside the hub). 200 deg of travel covers EL -5 to 185, 227 kg.cm.
  The gearbox is visible through the translucent 96 mm lens on the outer face of the drum.
- **EL bearing, left**: a 30 mm stub axle bolts to the hub's left face (4x M4x20) and turns in a split printed
  bushing (36/30.5 x 40) inside the left drum; the left axis cap carries the AS5600 on a column 2 mm off the magnet
  in the stub end, behind a translucent status window.
- **Cradle**: rounded 90 mm hollow hub (12 mm vertical, 3 mm top fillets) with a flush lid (4x M3x12 into nuts in a
  12 mm rim). All flange bolts end in nuts inside the cavity. Boom and counterweight arm are 40 mm round tubes with
  round 70 mm root flanges (4x M4x20 on a 42 mm square) and round spigots into the hub.
- **Dish interface**: boom ends in a 64 mm flange with four pocketed M5 nuts; bolt on the slotted plate adapter or the
  1.5 in pipe stub with 4x M5x12.
- **Counterweight**: opaque black canister (~1.3 L) on the square arm with a row of holes for balance adjustment;
  orange press cap with nuts pocketed in its plug.
- **Vendor CAD**: `vendor/3215-0001-000{4,9}/` hold goBILDA's STEP assemblies (120 MB each); the model uses measured
  envelopes (`STINGRAY` in `gimbal/parts.py`) split into a static part and an output part per axis.

## FEA
`freecadcmd -c "import runpy; runpy.run_path('tools_fea.py')"` writes `out/fea/`: the printed structure rebuilt without
fastener features and fused into one bonded solid (`Perigee_FEA_structure.step` / `.brep`, plus one STEP per rigid
group), and `Perigee_FEA.FCStd` with a FEM analysis set up: PLA material (stiffness derated for 30 % gyroid), the
foot fixed, dish weight and wind on the boom flange, counterweight on the arm end, gravity, and a CalculiX static
solver. The file already contains a second-order tetrahedral mesh (gmsh, 2.5 to 9 mm, about 450 k nodes) and
`out/fea/ccx/` holds the written CalculiX input deck.

gmsh is not in the Arch repos; the project venv has it (`.venv/bin/gmsh`, installed with pip). To re-mesh from the
FreeCAD GUI point Preferences > FEM > Gmsh at that binary, or edit the sizes in `tools_fea.py` and rerun. The solver
is in the AUR: `yay -S calculix-ccx`. Then either open the FCStd, double-click CalculiX and Run, or run
`ccx -i Perigee_FEA` inside `out/fea/ccx/` and load the `.frd` result. Two sliding fits (column spigots, hub
spigots) and the gearbox blocks and shafts are bonded PLA bridge solids in this model, so deflection at the joints is
overestimated; the canister is left out and its load applied on the counterweight arm.

### Parametric sweep (dish size x mass x wind x elevation)
`freecadcmd -c "import runpy; runpy.run_path('tools_fea_poses.py')"` rebuilds the bonded structure with the cradle
rotated to EL 0 / 45 / 90 (`out/fea/sweep/pose_*.brep`). Then `.venv/bin/python tools_fea_sweep.py mesh | solve | post`:
gmsh meshes each pose (~140 k nodes, straight-sided C3D10), CalculiX solves 9 unit load steps per pose (Fx Fy Fz Mx My Mz
on a rigid coupling at the boom flange, Fz Mx at the counterweight arm end, gravity; ~2 h per pose, run the three in
parallel), and `post` superposes them over the scenario grid in `tools_fea_sweep.py` (0.6-1.2 m dishes, 3-15 kg,
0-40 m/s head-on and side wind) in seconds, writing `out/fea/sweep/SWEEP.md` and `sweep.csv` (peak / 99.9th percentile
von Mises, safety factor, dish deflection, pointing error, counterweight needed, unbalanced EL moment). The `.frd` files
are ~400 MB each and are needed only to re-run `post`.

## Print notes
- ASA or PETG, 0.2 mm layers, 4-5 walls, 30-40 % gyroid for structural parts (base, puck, arms, hub, trunnions).
- Columns print lying on their outer side (as exported); the stencil letters then print without support.
- Lid and counterweight cap print face down; their PERIGEE logos are engraved so that works.
- Ball grooves are 45 deg V's and print clean. Sand lightly, then PTFE grease.
- Nut pockets are 7 mm AF (M4) and 5.5 mm AF (M3) with 0.4 to 0.5 mm clearance; nut traps are the same width, opening on a side face.

## Assembly order (matches the animation)
1. Base: Stingray-4 onto the four floor pillars, 4x M4x40 up from under the base into its grid holes; feed its
   cable out through the floor slot.
2. Grease the thrust face and bore, lower the puck onto the gearbox standoffs, 4x M4x25 down from the deck into the
   standoffs, bolt the retainer ring (12x M4x20, nuts in the pedestal wall).
3. Housing: slide nuts into the pillar traps, 4x M4x30 up from under the puck; electronics in; yoke plate 4x M4x16
   into the upper pillar traps. Lower columns 4x M4x25 each from under the plate into the foot traps (before the
   plate goes on). Upper columns: spigot into the socket, 2x M4x50 across + nuts.
4. Right drum: slide the Stingray-9 into its pocket from the inner face; 4x M4x50 and 4x M4x20 from the drum surface
   into its side threads. Translucent lens 4x M3x12 into the cavity-loaded nuts (two of them sit in the cap).
5. Cradle: with the hub lid off, bolt the left stub (4x M4x20), boom and counterweight arm flanges (4x M4x20 each)
   with nuts inside the hub; magnet in the stub end. Lower bushing half into the left shoulder, grease, drop the
   cradle in so the right wall meets the Stingray-9 standoffs, 4x M4x12 from inside the hub into the standoffs, lid
   4x M3x12. Upper bushing half, caps (2x M4x25 each into the shoulder traps).
6. Left shoulder: AS5600 on the cap column (2x M3x8 + nuts), cap 4x M3x12, press the window in.
7. Drop the 4 M5 nuts into the pockets on the back of the boom flange, adapter 4x M5x12. Canister collar 2x M4x55 +
   nuts; drop the 2 M3 nuts into the pockets in the cap's plug, push the cap on, 2x M3x10 through skirt and wall.
   Balance with the canister empty, then fill.

## Caveats you should know
- Both Stingrays must be the **feedback-mode** variants (3215-0001-xxxx) and driven by a controller that can produce
  the wide PWM range (500-2500 us) they need for their full 450 / 200 deg travel; the Uno sketch has to unwrap
  azimuth across the 1.25-turn range and read the feedback wires.
- The right EL bearing is the Stingray-9's own hub-shaft bearings; keep the payload balanced with the counterweight so
  the gearbox sees torque, not a permanent overhung load.
- Printed plain bearings need grease and a break-in; the AZ journal clearance is 0.5 mm radial and the retainer play
  0.8 mm. If the axis is stiff, sand the puck lip lightly. Replace the EL bushing when it wears.
- The file is written headless. FreeCAD's GUI opens such a file with every object hidden, so the build injects a
  `GuiDocument.xml` (`tools_guidoc.py`: visibility, filament colours, transparency) into the master and per-part files.
  Still run `PerigeeColors.FCMacro` once after opening: it attaches the Assembly workbench view providers to the joints,
  the Exploded_Assembly view and its steps, and the simulation. Until then double-clicking Exploded_Assembly does nothing.
- The Stingray placeholders are measured envelopes, not the vendor geometry; open the STEP files in `vendor/` when
  you need a detail (they are heavy).
- The Spreadsheet in the master file documents the parameters; it is regenerated by the script, not live-linked.
