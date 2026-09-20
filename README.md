# PERIGEE gimbal

Fully 3D-printed azimuth/elevation pedestal for a dish up to 1 m (mesh), 15-20 lb payload, driven by two
high-torque servos through printed spur reductions, closed-loop on AS5600 magnetic encoders, controlled by an
Arduino Uno on the rotating head (power through a capsule slip ring, Bluetooth serial to the PC running Perigee).
No bearings of any kind: both axes run on greased printed plain bearings. The only non-printed parts are metric
screws, nuts, heat-set inserts, the slip ring, servos and electronics.

Everything here is generated from `params.json` by `build.py` running inside FreeCAD 1.1 (headless).

```
./build.sh                              # regenerate everything (about 80 s): parts, assembly, exploded view, audit, exports, viewer bundle
.venv/bin/python tools_render.py        # OpenGL inspection renders into out/renders (moderngl on headless EGL, ~1 s)
.venv/bin/python tools_render.py --anim # cinematic assembly film -> out/renders/assembly.mp4 (30 fps, ~2 min)
.venv/bin/python tools_render.py --spot 1.5,9.5,36   # contact sheet of film frames at those seconds
python3 -m venv .venv && .venv/bin/pip install moderngl numpy pillow   # one-time renderer setup
```

The build also runs an **overlap audit** over every pair of parts (`out/overlap_audit.txt`, must say `clean`) and
creates an **Exploded_Assembly** view (Assembly workbench > Exploded Views) with one move per part in assembly order,
so FreeCAD itself can animate the explosion with its slider.

## Hardware is real, and nothing threads into plastic
Every screw, nut and washer is modelled (about 210 pieces) as App::Link instances of master parts in
`Hardware_Library`, placed with the same coordinates that cut their counterbores, nut pockets and nut traps.
All fasteners are ISO 4762 socket head cap screws in M3 / M4 / M5 with ISO 4032 hex nuts. No heat-set inserts and
no self-tapping: each bolt ends in a nut that sits either in a blind hex pocket on the far face (where that face is
reachable at assembly time) or in a side-loaded nut trap (a slot the nut slides into before the mating part goes on:
pedestal wall, ring gear, pillar feet, arm feet, shoulder tops, around the bores, servo pocket, hub walls). The hub is
a hollow box with a bolted lid so all its flange nuts sit inside. The BOM lists exact counts per size and length.
Tools: 2.5 / 3 / 4 mm hex keys and 5.5 / 7 / 8 mm spanners.

## Styling
Modelled on the CMG-ISS EO/IR gimbal (and the other screenshots in ~/screenshots): black pedestal with a wide flange
plate, white azimuth housing under a broad rounded yoke plate with a round access cover, white arms swelling into
rounded shoulders at the elevation axis with small screwed caps, payload between the arms. Filaments: white (housing,
plate, arms, caps, bushings, clamps), black (pedestal, flange plate, hub, stubs, boom, counterweight arm), grey (small
covers), orange (gears, counterweight cap), translucent (canister, window, light pipe).

## What you get

| Path | Contents |
|---|---|
| `out/PerigeeGimbal.FCStd` | Master file: Spreadsheet `Params`, Assembly `Gimbal` with 7 rigid groups, 10 joints (AZ + EL revolutes, 4 pinion revolutes, 4 gear couplings), a `Track_Pass_Simulation`, and 9 TechDraw pages |
| `out/parts_fcstd/*.FCStd` | One FreeCAD file per printed part, already in print orientation |
| `out/step/*.step` | STEP per printed part (world coordinates) |
| `out/stl/*.stl` | STL per printed part, oriented for the Bambu A1 bed (all fit 256 mm) |
| `out/drawings/*.dxf` | Drawing pages as DXF (the same pages live inside the FCStd with the views) |
| `out/BOM.md` | Printed parts with filament colour and mass estimate, hardware list, key numbers |
| `out/manifest.json` | Machine-readable part list (group, colour, print bbox, volume) |
| `viewer/` | Interactive three.js viewer (`index.html` + `meta.json` + `parts_bundle.json`); pass simulation, fastener toggle, and an **Assemble** mode that builds the gimbal from the base up with a scrub bar |
| `out/renders/assembly.mp4`, `.gif` | Rendered assembly animation: 18 shots, each sub-assembly framed close from its best angle, 360 spins after the yoke and at the end, every fastener arriving along its own axis. Shot list = `SHOTS` in `build.py`, shared with the viewer |
| `macros/PerigeeColors.FCMacro` | Run once in the FreeCAD GUI: applies filament colours / transparency, hides joint markers |
| `macros/PerigeeAnimate.FCMacro` | Run in the GUI: animates an overhead pass with flip-over EL (no solver needed) |

### Animation inside FreeCAD
Open the master file, run `PerigeeColors.FCMacro`, then either run `PerigeeAnimate.FCMacro`, or open
Assembly workbench > Simulation > `Track_Pass_Simulation`, press *Generate* then *Play*. The simulation drives the AZ
and EL joints with formulas (`18*time` and a harmonic 0-180 deg sweep) and the gear joints spin the pinions.

## Design summary (v5: CMG-ISS style, plain bearings)

- **Frame**: Z up, AZ axis = Z, EL axis = X at z = 580 mm (510 mm above the deck). Boresight at EL 0 is +Y.
- **Pedestal**: black 232 mm pedestal body on a 252 mm foot flange (6x M6 for a mount plate); a black 252 mm flange
  plate on top is the AZ retainer (12x M4x12) and the visible rotation seam, wider than the white housing above it.
- **AZ bearing**: printed plain bearing. The puck's 15 mm lip (OD 207) is the journal in the 208 mm bore; the step at
  z 40 (r 92-104) is the greased thrust face; the flange plate traps the lip with 0.8 mm play.
- **AZ housing**: the rotating electronics bay is a white drum of the same 232 mm diameter as the pedestal body, so the
  column reads as one shape with the black flange plate between; four internal pillars tied to the wall by ribs
  carry the yoke (M4 top and bottom, nuts trapped in the pillars). Its lid is the broad 240 x 170 yoke plate (34 mm corner radii) with a round bolted access
  cover; the arms bolt to the plate from below. PERIGEE stencil and light pipe on the back.
- **AZ drive**: servo sits in a pocket in the puck, pinion (20 T, module 2) hangs below and rolls inside a fixed
  internal ring gear (84 T) bolted to the base floor. Ratio 4.2:1. Continuous rotation.
- **AZ encoder**: fixed sun (36 T) -> idler (16 T) -> sensor gear (36 T) under the deck. Sun and sensor have the same
  tooth count so the sensor gear turns exactly once per head turn. AS5600 sits in a well in the deck 1-2 mm above the
  magnet in the sensor gear's pin.
- **Slip ring**: 22 mm capsule in the centre tower, stub clamped in the puck. Power only; the Uno, Bluetooth,
  buck converter and SDR live in the electronics bay on the head.
- **Yoke**: two 40 mm slab arms with rounded edges (taper clipped to the 15 deg dish-sweep cone), each printed as a
  lower segment and a shoulder segment (tenon + 2x M4x40). The shoulders are 64 mm thick rounded blocks around the
  elevation axis, split at the axis as pillow blocks with rounded bolted caps. Stencil-cut PERIGEE on the lower
  segments, cut per side so both read from outside.
- **EL bearing**: two 30 mm stub axles bolt to the hub faces (4x M4x16 each) and turn in split printed bushings
  (36/30.5 x 40) inside the shoulders; the cradle drops in from above before the caps go on.
- **EL drive**: 72 T gear bolted to the right hub face (4x M4x16), inboard of the arm. The servo sits inside the
  right shoulder in a pocket behind a small round cover (6x M3x10, with an integral spacer that holds the servo);
  its shaft passes through the shoulder wall to the 20 T pinion. Ratio 3.6:1, EL range -5 to 185 deg.
- **EL encoder and caps**: small on-axis caps on both shoulders (6x M3x10). The left cap carries the AS5600 on a
  column 2 mm off the magnet in the stub end, behind a translucent status window.
- **Cradle**: faceted 90 mm hub with bushing bores both sides. Boom and counterweight arm have chamfered 64 mm root
  flanges, 4x M4x10 each into hub inserts.
- **Dish interface**: boom ends in a 64 mm flange; bolt on the slotted plate adapter or the 1.5 in pipe stub.
- **Counterweight**: translucent canister (~1.3 L) on a square arm with a row of holes for balance adjustment.

## Print notes
- ASA or PETG, 0.2 mm layers, 4-5 walls, 30-40 % gyroid for structural parts (base, puck, arms, hub, trunnions).
- Gears 100 % infill, 0.16 mm layers. Print the ring gear teeth-up.
- Arms print lying on their outer face (as exported), so the honeycomb and letters print without support.
- Lid and counterweight cap print face down; their PERIGEE logos are engraved so that works.
- Ball grooves are 45 deg V's and print clean. Sand lightly, then PTFE grease.
- Nut pockets are 7 mm AF (M4) and 5.5 mm AF (M3) with 0.4 to 0.5 mm clearance; nut traps are the same width, opening on a side face.

## Assembly order (matches the animation)
1. Base: 6x M4x10 up through the floor into the ring gear inserts; sun gear onto the tower D-flat with 2x M3x12;
   slip ring capsule flange 2x M3x8; glow ring halves into the groove.
2. Head module: idler on its M4x45 axle (nut in the deck pocket), sensor gear with magnet + keeper bar (2x M3x8),
   AS5600 in the well (2x M3x6), AZ pinion on the servo (M3x10 axial), servo into the pocket, clamp (2x M3x8),
   hex clamp onto the slip ring stub (M3x16 pinch).
3. Grease the thrust face and bore, lower the puck in, bolt the retainer ring (12x M4x12).
4. Housing: slide nuts into the pillar traps, 4x M4x30 up from under the puck; electronics in; yoke plate 4x M4x16
   into the upper pillar traps. Arm lower segments 4x M4x25 each from under the plate into the foot traps (before the
   plate goes on). Shoulder segments: tenon + 2x M4x40 + nuts.
5. Cradle: with the hub lid off, bolt the stubs (4x M4x20 each), EL gear (4x M4x20), boom and counterweight arm
   flanges (4x M4x20 each) with nuts inside the hub; magnet in the left stub end; lid 4x M3x20. Slide the axis-cap
   nuts into the bore slots, lower bushing halves into the shoulders, grease, drop the cradle in, upper halves,
   caps (2x M4x20 into the shoulder traps).
6. Right shoulder: nuts into the pocket-wall slots, pinion onto the servo shaft (M3x10 into the servo's own thread),
   servo into the pocket, servo cover 4x M3x12, axis cap 4x M3x12. Left shoulder: AS5600 on the cap column
   (2x M3x10 + nuts), cap 4x M3x12, press the window in.
7. Adapter 4x M5x20 + nuts. Canister collar 2x M4x55 + nuts, cap 2x M3x10 + nuts inside the rim.
   Balance with the canister empty, then fill.

## Caveats you should know
- **Both servos must be continuous-rotation types** (or converted): the AZ axis is continuous, and with a 5:1 EL
  reduction a 300 deg positional servo would only give 60 deg of elevation.
- Printed plain bearings need grease and a break-in; the AZ journal clearance is 0.5 mm radial and the retainer play
  0.8 mm. If the axis is stiff, sand the puck lip lightly. Replace the EL bushings when they wear.
- The Exploded_Assembly view is created by script and has not been exercised in the GUI on this machine (no display). The Arduino closes the position loop from
  the AS5600s. If you insist on positional servos on EL, set `el_gear_teeth` to 32 (1.6:1) and rebuild.
- The servo holder is a generic 40 x 40 x 50 mm box (`servo_w/h/len`). Set the real numbers and rebuild before printing
  the puck, sleeve and pinions.
- The Spreadsheet in the master file documents the parameters; it is regenerated by the script, not live-linked.
- TechDraw pages have orthographic + isometric views with annotations; add dimensions in the GUI where you need them.
