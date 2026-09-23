# Perigee gimbal FEA, first static case (2026-09-21)

Model: bonded, defeatured structure (tools_fea.py), 215 397 second-order tets / 372 178 nodes, CalculiX 2.23 iterative
Cholesky, converged (14 min on 16 threads). Material: PLA derated for 30 % gyroid, E = 2.1 GPa, yield 30 MPa.
Loads: foot fixed; 88 N dish weight and 130 N horizontal wind at the boom flange; 62 N counterweight at the arm end; gravity.

| Result | Value |
|---|---|
| Peak von Mises | 2.68 MPa at the boom root, just outboard of its hub flange (z 580) |
| 99.9th percentile von Mises | 2.09 MPa |
| Peak by region | columns 2.39, counterweight arm 2.03, drums 1.61, hub+boom 1.38, pedestal 0.86 MPa |
| Safety factor to PLA yield | about 11 |
| Deflection at the EL axis (drums) | 2.65 mm |
| Deflection at the boom flange | 2.9 mm |
| Deflection at the counterweight arm end | 3.6 mm |

Reading: stresses are an order of magnitude below yield everywhere; the structure is stiffness-limited, not strength-limited.
Almost all of the 2.9 mm at the dish is the two columns bending under the 130 N wind (2.65 mm already at the axis), i.e.
about 0.25 deg of tilt at the head, far inside a 1 m dish's beamwidth. The boom itself adds only 0.3 mm over its 158 mm.
Caveats: gearbox blocks and shafts are modelled as PLA (real ones are stiffer), sliding fits are bonded, and the wind is a
single static gust with no dynamic factor; treat deflections as +-30 %.
