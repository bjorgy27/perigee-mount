# PERIGEE gimbal

Fully 3D-printed azimuth/elevation pedestal for a dish up to 1 m (mesh), 15-20 lb payload, driven directly by two
goBILDA Stingray servo gearboxes (Stingray-4 on azimuth, 450 deg of travel; Stingray-9 on elevation, 200 deg), both
run open loop: they are positional servos, the pulse width is the position, and there is no encoder or feedback wire.
Controlled by an ST Nucleo-F401RE on the rotating head (USB serial to the PC running Perigee). Azimuth needs no slip ring: 1.25 turns
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
pedestal wall, pillar feet, arm feet, shoulder ribs, around the left bore, lens window, hub rim), or in one of the Stingray-9's own
threads (3x M4 along the axis into the tapped holes in the back face of its aluminium block; its 16 mm-grid holes are
plain 4 mm holes, not tapped). The hub is
a hollow box with a bolted lid so all its flange nuts sit inside. The BOM lists exact counts per size and length.
Tools: 2.5 / 3 / 4 mm hex keys and 5.5 / 7 / 8 mm spanners.

## Styling
Canadarm / R.O.B. proportions: black pedestal with a wide flange plate, white azimuth housing under a plain round
yoke plate, two slender 60 mm round columns rising to 120 mm shoulder drums on the elevation axis (a half-circle cap over a
stadium-shaped lower half; both are 4 mm hollow shells open toward the hub), a
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

## Design summary (v9: Stingray direct drives, round columns, hollow 120 mm shoulder drums)

- **Frame**: Z up, AZ axis = Z, EL axis = X at z = 599 mm (510 mm above the deck). Boresight at EL 0 is +Y.
- **Pedestal**: black 216 mm pedestal body on a 236 mm foot flange (6x M6 for a mount plate), 75 mm tall; a black
  236 mm flange plate on top is the AZ retainer (split in two halves since v10: 8x M4x20 hooks in the pedestal, 4x
  M4x20 at the two lap notches) and the visible rotation seam. Everything fits a 256 mm
  bed with room for a brim.
- **AZ bearing**: printed plain bearing. The puck's 15 mm lip (OD 191) is the journal in the 192 mm bore; the step at
  the puck underside (r 84-96) is the greased thrust face; the flange plate traps the lip with 0.8 mm play. The
  Stingray-4's own bearings centre the shaft; the plain bearing carries the weight and moments.
- **AZ drive**: goBILDA Stingray-4 (3215-0001-0004, feedback mode) stands on four pillars on the pedestal floor,
  bolted from below (4x M4x40 into its 16 mm grid). Its output gear faces up and its four M4 standoffs bolt to the
  underside of the puck (4x M4x25 from the deck). 450 deg of travel, 100 kg.cm, 15 rpm. Open loop: the
  commanded pulse is the position. Cables pass through a hole in the puck into the pedestal annulus as a service loop.
- **AZ housing**: the rotating electronics bay is a white drum of the same 216 mm diameter as the pedestal body; four
  internal pillars tied to the wall by ribs carry the yoke (M4 top and bottom, nuts trapped in the pillars). Its lid is
  an oval 244 x 220 mm yoke plate with a translucent round access cover; the columns bolt to the plate from below and
  their cables pass up through it.
- **Yoke**: two hollow 60 mm round columns (3 mm walls), each printed as a lower segment and an upper segment joined
  by a solid spigot with 2x M4x50 across; the upper segment carries the lower half of a 120 mm shoulder drum on the
  elevation axis. The drum profile is a half-circle above the axis and a stadium below it (straight sides for 20 mm,
  then a half-circle: the Stingray-9 hangs 70 mm below the axis and the columns' foot pattern is fixed by the printed
  yoke plate, so the lower half has to reach 80 mm down). Both halves are 4 mm shells closed on the outer face and open
  toward the hub, with the internal ribs, channel and pillow block tied to the outer plate; they print lying on their
  flat outer face with no support. The caps have a 4 mm split-plane plate and bolt down with 2x M4x50 each into nuts
  36 mm deep in ribs of the lower half (traps open on the drum side). Feet: 4x M4x25 up from under the yoke plate into
  side-loaded nuts in a solid foot boss (unchanged from v8, so the printed plate still fits).
- **EL drive**: goBILDA Stingray-9 (3215-0001-0009, feedback mode) slides into a 3 mm-walled channel inside the right
  shoulder shell from the inner face, servo below the axis, and is screwed along its axis with 3x M4x20 through a
  12 mm back plate into the three tapped holes in the back face of its block (on a 22.6 mm circle around the shaft: one
  11.3 mm above the axis, two either side of it on the split plane), all driven with a 3 mm hex key through the 76 mm
  window on the outer face and a 36 mm sight bore before the lens goes on (v11; v10 screwed into the 16 mm-grid holes,
  which turned out to be plain, untapped 4 mm holes). The hub-shaft end and its snap ring stick 3.6 mm out of the back
  of the block into a 15 mm pocket in the back plate. Its 88 mm output gear turns in the gap between drum and hub and its four M4 standoffs bolt
  straight to the hub's right wall (4x M4x12 from inside the hub). 200 deg of travel; the firmware uses EL -2 to 185 of it, 227 kg.cm.
  The gearbox, its screws and the servo are visible through the translucent 88 mm lens, centred on the axis.
- **EL bearing, left**: a 30 mm stub axle bolts to the hub's left face (4x M4x20) and turns in a split printed
  bushing (36/30.5 x 40) inside the left drum; a plain left axis cover with a translucent status window closes the
  bore.
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
- Shoulder halves and caps print on their flat outer face (as exported): the shells are open toward the hub, which is
  the top of the print, so there is nothing to bridge or support. They are all walls, so print them at 100 % (the mass
  estimate in the BOM, which assumes 45 % effective density, undercounts them: expect about 1.07 g per cm3).
- Lid and counterweight cap print face down; their PERIGEE logos are engraved so that works.
- Ball grooves are 45 deg V's and print clean. Sand lightly, then PTFE grease.
- Nut pockets are 7 mm AF (M4) and 5.5 mm AF (M3) with 0.4 to 0.5 mm clearance; nut traps are the same width, opening on a side face.

## Assembly order (matches the animation)
1. Base: Stingray-4 onto the four floor pillars, 4x M4x40 up from under the base into its grid holes; feed its
   cable out through the floor slot.
2. Hooks: slide 8 M4 nuts into the pedestal wall traps at 45 / 75 / 105 / 135 deg and their mirror images (the four
   traps next to the X axis stay empty). Lay both retainer halves on the empty pedestal in their final place, drop an
   M4x20 into each of the 8 slots and snug it onto the groove floor, then back each off about a quarter turn and
   slide the halves back out along +-Y. The screw heads are now hooks at exactly the right height.
3. Head: on the bench, slide nuts into the drum's pillar-foot traps, set the drum on the puck, 4x M4x30 up from under
   the puck. Grease the thrust face and bore, lower the puck + drum onto the gearbox standoffs (the lip passes inside
   the hooks), 4x M4x25 down from the deck into the standoffs, reached through the open top of the drum.
4. Retainer: slide half A in from +Y through the gap under the drum (its slots pass the hooks, its thin tongues end
   past the X axis), then half B from -Y with its tongues over A's. Push an M4 nut up into each of the 4 hex pockets
   under A's tongues (they sit outside the pedestal, so a fingertip reaches them) and drive 4x M4x20 down through the
   notches. Do this before the yoke plate goes on: the plate overhangs the notches in X.
5. Housing: electronics in; yoke plate 4x M4x16
   into the upper pillar traps. Lower columns 4x M4x25 each from under the plate into the foot traps (before the
   plate goes on). Upper columns: spigot into the socket, 2x M4x50 across + nuts.
6. Right drum: slide the Stingray-9 into its channel from the inner face (cap off), shaft end into the pocket in the
   back plate; through the window, 2x M4x20 into the two back-face threads on the split plane (half holes; the cap's
   halves slide down over them later); the M4x20 above the axis goes in after the cap (step 7). Then the translucent lens, 4x M3x12 into the window-loaded nuts (two of them sit in the cap).
7. Cradle: with the hub lid off, bolt the left stub (4x M4x20), boom and counterweight arm flanges (4x M4x20 each)
   with nuts inside the hub. Lower bushing half into the left shoulder, grease, drop the
   cradle in so the right wall meets the Stingray-9 standoffs, 4x M4x12 from inside the hub into the standoffs, lid
   4x M3x12. Upper bushing half, caps (2x M4x50 each into the rib traps), then the last M4x20 into the Stingray-9.
8. Left shoulder: axis cover 4x M3x12, press the window in.
9. Drop the 4 M5 nuts into the pockets on the back of the boom flange, adapter 4x M5x12. Canister collar 2x M4x55 +
   nuts; drop the 2 M3 nuts into the pockets in the cap's plug, push the cap on, 2x M3x10 through skirt and wall.
   Balance with the canister empty, then fill.

## Caveats you should know
- Both Stingrays must be the **feedback-mode** variants (3215-0001-xxxx) and driven by a controller that can produce
  the wide PWM range (500-2500 us) they need for their full 450 / 200 deg travel. "Feedback mode" only means
  positional mode: there is no feedback wire. The firmware (perigee-control/firmware/perigee_mount_stm32) keeps azimuth
  inside 0..400 deg of the 1.25-turn cable loop and elevation inside -2..185.
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
