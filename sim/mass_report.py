"""Printed-part mass estimate from the CAD STLs (PETG, walls + infill), compared with the mass SolidWorks reports.

Reads sim/meshes/main - <part>.STL (metres, one per printed part) and build/zero_pose.json (SolidWorks mass per part).
Model: shell = surface area x shell thickness (walls and top/bottom skins), interior = (volume - shell) x infill.
Run from sim/:  python mass_report.py [--walls 4 --skins 5 --infill 0.25 --layer 0.2 --line 0.45 --density 1.27]
"""
import argparse, json
from pathlib import Path
import trimesh

HERE = Path(__file__).parent
PARTS = ["base cover^main-1", "base plate^main-2", "J2 lower arm^main-1", "J3 upper arm^main-1",
         "Servo bottom-1", "Servo arm 2 con-1", "J5 yoke^main-1", "head front^main-1"]
LINK = {"base cover^main-1": "base", "base plate^main-2": "base", "Servo bottom-1": "turntable",
        "J2 lower arm^main-1": "lower_arm", "J3 upper arm^main-1": "upper_arm",
        "Servo arm 2 con-1": "roll", "J5 yoke^main-1": "yoke", "head front^main-1": "head"}

ap = argparse.ArgumentParser()
ap.add_argument("--walls", type=int, default=4)
ap.add_argument("--skins", type=int, default=5, help="top/bottom layers")
ap.add_argument("--infill", type=float, default=0.25)
ap.add_argument("--layer", type=float, default=0.2, help="mm")
ap.add_argument("--line", type=float, default=0.45, help="extrusion width, mm (0.4 nozzle)")
ap.add_argument("--density", type=float, default=1.27, help="g/cm3 (PETG)")
a = ap.parse_args()

wall_t = a.walls * a.line                      # mm, side walls
skin_t = a.skins * a.layer                     # mm, top/bottom
shell_t = (2 * wall_t + skin_t) / 3            # orientation unknown: blend (two side axes, one vertical)
sw = json.load(open(HERE / "build" / "zero_pose.json"))["mass"]

print(f"PETG {a.density} g/cm3 | {a.walls} walls ({wall_t:.2f} mm), {a.skins} skins ({skin_t:.2f} mm), "
      f"{a.infill:.0%} infill | effective shell {shell_t:.2f} mm\n")
print(f"{'part':<22}{'vol cm3':>9}{'area cm2':>10}{'solid g':>9}{'printed g':>11}{'ratio':>7}{'SW kg':>8}  watertight")
per_link, tot_solid, tot_print, printed_kg = {}, 0.0, 0.0, {}
for p in PARTS:
    f = HERE / "meshes" / f"main - {p}.STL"
    m = trimesh.load(f, force="mesh")
    vol, area = abs(m.volume) * 1e6, m.area * 1e4                 # m3 -> cm3, m2 -> cm2
    shell = min(area * shell_t / 10, vol)                          # cm2 * mm / 10 = cm3
    solid_g = vol * a.density
    print_g = (shell + (vol - shell) * a.infill) * a.density
    printed_kg[p] = round(print_g / 1000, 6)
    per_link[LINK[p]] = per_link.get(LINK[p], 0) + print_g
    tot_solid += solid_g; tot_print += print_g
    print(f"{p:<22}{vol:9.1f}{area:10.1f}{solid_g:9.1f}{print_g:11.1f}{print_g/solid_g:7.0%}{sw[p]['mass']:8.3f}  {m.is_watertight}")
print(f"\n{'TOTAL printed parts':<22}{'':>9}{'':>10}{tot_solid:9.1f}{tot_print:11.1f}{tot_print/tot_solid:7.0%}")
print("\nprinted mass per link (g, servos not included): " + ", ".join(f"{k} {v:.0f}" for k, v in per_link.items()))
json.dump(printed_kg, open(HERE / "build" / "printed_mass.json", "w"), indent=1)
print("wrote build/printed_mass.json (read by build/gen_urdf.py)")
print("Estimate only: slicer adds supports/brim/purge, and real prints run a few % heavier. Check one part on a scale.")
