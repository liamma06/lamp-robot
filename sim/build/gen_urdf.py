"""Build lamp.urdf from build/zero_pose.json (SolidWorks zero pose = CAD pose with J1 at 90 deg).
SolidWorks is Y-up; the URDF is rotated to Z-up. Link frames are world-aligned at each joint pivot.
Meshes: sim/meshes/*.STL and sim/meshes/servos/*.STL (one STL per CAD part, exported from SolidWorks in assembly coordinates, metres).
They are merged per link into sim/meshes/links/ (visual + convex-hull collision)."""
import json, math, re
from pathlib import Path
import numpy as np
import trimesh

HERE = Path(__file__).parent

SERVO_KG = 0.055          # real STS3215 mass
EFFORT, VEL = 2.94, 4.5   # N*m (STS3215 stall at 12 V, ~30 kg*cm), rad/s
Q = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)   # SW (x,y,z) -> URDF (x,-z,y)

r = json.load(open(HERE / "zero_pose.json"))
tf, mass = r["zero_tf"], r["mass"]

LINKS = {  # link: components
    "base": ["base cover^main-1", "base plate^main-2", "STS3215_c.step-1"],
    "turntable": ["Servo bottom-1", "STS3215_c.step-2"],
    "lower_arm": ["J2 lower arm^main-1"],
    "upper_arm": ["J3 upper arm^main-1", "STS3215_c.step-3", "STS3215_c.step-4"],
    "roll": ["Servo arm 2 con-1", "STS3215_c.step-5"],
    "yoke": ["J5 yoke^main-1", "STS3215_c.step-6"],
    "head": ["head front^main-1"],
}
# joint: (parent, child, mate-axis source, limits deg relative to zero)
JOINTS = [
    ("J1", "base", "turntable", "Concentric1", "J1 angle", (-150, 150)),
    ("J2", "turntable", "lower_arm", "Concentric3", "J2 angle", (-60, 42)),   # +42: lower arm hits base at +46.5 (measured in CAD), ~4 deg margin
    ("J3", "lower_arm", "upper_arm", "Concentric4", "J3 angle", (-30, 140)),
    ("J4", "upper_arm", "roll", "Concentric6", "J4 angle", (-170, 170)),
    ("J5", "roll", "yoke", "Concentric8", "J5 angle", (-70, 70)),
    ("J6", "yoke", "head", "Concentric9", "J6 angle", (-170, 170)),
]
REP = {"J1 angle": "Servo bottom-1", "J2 angle": "J2 lower arm^main-1", "J3 angle": "J3 upper arm^main-1",
       "J4 angle": "Servo arm 2 con-1", "J5 angle": "J5 yoke^main-1", "J6 angle": "head front^main-1"}

def is_servo(n): return n.startswith("STS3215")

# pivots and axes (SW frame)
pivot, axis = {}, {}
for j, par, ch, mate, key, lim in JOINTS:
    pts = [np.array(e["pt"]) for e in r["axes_at_zero"][mate]]
    pivot[j] = sum(pts) / len(pts)
    # positive direction = rotation sense of a +5 deg change of that joint's angle mate (measured in CAD)
    ax = np.array(r["perturb"][key][REP[key]]["axis"])
    axis[j] = ax / np.linalg.norm(ax)
    if abs(axis[j][2]) > 0.9:   # slide the pivot along the axis to the lamp mid-plane (SW z = 0)
        pivot[j] = pivot[j] - axis[j] * pivot[j][2] / axis[j][2]

# link mass properties in SW frame about link COM
link = {}
for name, comps in LINKS.items():
    ms, coms, Is = [], [], []
    for c in comps:
        m = mass[c]["mass"]; I = np.array(mass[c]["I_com_local"]).reshape(3, 3)
        if is_servo(c): I = I * SERVO_KG / m; m = SERVO_KG
        R = np.array(tf[c]["R"]); t = np.array(tf[c]["t"])
        ms.append(m); coms.append(R @ np.array(mass[c]["com_local"]) + t); Is.append(R @ I @ R.T)
    M = sum(ms); com = sum(m * c for m, c in zip(ms, coms)) / M
    I = sum(Ii + m * ((np.dot(c - com, c - com)) * np.eye(3) - np.outer(c - com, c - com)) for Ii, m, c in zip(Is, ms, coms))
    link[name] = (M, com, I)

origin = {"base": np.zeros(3)}
for j, par, ch, *_ in JOINTS: origin[ch] = pivot[j]

def fmt(v): return " ".join(f"{x:.9f}" for x in v)
def rpy_z_to(v):  # rotation taking +z to unit v (URDF frame) as rpy
    v = v / np.linalg.norm(v); z = np.array([0, 0, 1.0]); c = np.cross(z, v); s = np.linalg.norm(c)
    if s < 1e-9: return np.zeros(3) if v[2] > 0 else np.array([math.pi, 0, 0])
    k = c / s; ang = math.atan2(s, np.dot(z, v)); K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    R = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
    return np.array([math.atan2(R[2, 1], R[2, 2]), math.asin(-R[2, 0]), math.atan2(R[1, 0], R[0, 0])])


# ---------------------------------------------------------------- meshes
MESH_DIR = HERE.parent / "meshes"
OUT_DIR = MESH_DIR / "links"
COLORS = {"base": "0.55 0.55 0.58", "turntable": "0.35 0.35 0.4", "lower_arm": "0.95 0.75 0.25", "upper_arm": "0.95 0.6 0.2",
          "roll": "0.3 0.3 0.35", "yoke": "0.9 0.45 0.25", "head": "0.95 0.95 0.9"}
comp_link = {c: ln for ln, cs in LINKS.items() for c in cs}

def stl_owner(stem):   # "main - STS3215_c.step-3 Top.step-1" -> "STS3215_c.step-3"
    m = re.match(r"(STS3215_c\.step-\d+) ", stem)
    return m.group(1) if m else stem

def com_world(c):
    return np.array(tf[c]["R"]) @ np.array(mass[c]["com_local"]) + np.array(tf[c]["t"])

def build_meshes():
    files = sorted(MESH_DIR.glob("main - *.STL")) + sorted((MESH_DIR / "servos").glob("main - *.STL"))
    by_link, unknown = {}, []
    for f in files:
        comp = stl_owner(f.stem[len("main - "):])
        (by_link.setdefault(comp_link[comp], []).append(f) if comp in comp_link else unknown.append(f.name))
    # The STLs may have been exported at a different J1 angle than the zero pose: fit the yaw about the vertical axis.
    single = [c for c in ("head front^main-1", "J2 lower arm^main-1", "J3 upper arm^main-1", "J5 yoke^main-1", "Servo arm 2 con-1")
              if (MESH_DIR / f"main - {c}.STL").exists()]
    d = []
    for c in single:
        m = trimesh.load(MESH_DIR / f"main - {c}.STL").center_mass; w = com_world(c)
        d.append((math.atan2(m[2], m[0]) - math.atan2(w[2], w[0]) + math.pi) % (2 * math.pi) - math.pi)
    phi = float(np.median(d)) if d else 0.0
    Ry = np.array([[math.cos(phi), 0, math.sin(phi)], [0, 1, 0], [-math.sin(phi), 0, math.cos(phi)]])
    resid = [np.linalg.norm(Ry @ trimesh.load(MESH_DIR / f"main - {c}.STL").center_mass - com_world(c)) * 1000 for c in single]
    print(f"meshes: {len(files)} STL files, yaw fix {math.degrees(phi):+.3f} deg, centroid-vs-CAD residual (mm): {np.round(resid, 2).tolist()}")
    if unknown: print("  meshes with no link (skipped):", unknown)
    missing = [c for c in comp_link if not any(stl_owner(f.stem[len("main - "):]) == c for f in files)]
    if missing: print("  CAD components with no mesh:", missing)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    result = {}
    for name, fs in by_link.items():
        parts = []
        for f in fs:
            m = trimesh.load(f)
            v = np.asarray(m.vertices) @ (Ry.T if name != "base" else np.eye(3))        # undo the J1 difference (base is fixed)
            v = (v - origin[name]) @ Q.T                                                 # into the URDF link frame
            parts.append(trimesh.Trimesh(v, m.faces, process=False))
        merged = trimesh.util.concatenate(parts)
        merged.export(OUT_DIR / f"{name}.stl"); merged.convex_hull.export(OUT_DIR / f"{name}_hull.stl")
        result[name] = (f"meshes/links/{name}.stl", f"meshes/links/{name}_hull.stl")
        print(f"  {name:10s} {len(fs):3d} parts, {len(merged.faces):6d} tris, hull {len(merged.convex_hull.faces)} tris")
    return result

mesh_files = build_meshes()

child_of = {par: ch for _, par, ch, *_ in JOINTS}
out = ['<?xml version="1.0"?>',
       '<!-- Generated by gen_urdf.py from build/zero_pose.json. Zero pose = SolidWorks CAD pose (J1=90, J2=60, J3=40, J4=90, J5=100 mate angles). -->',
       '<!-- Visual = merged CAD meshes per link; collision = convex hull of each link mesh. -->',
       '<robot name="lamp">']
for name in LINKS:
    M, com, I = link[name]
    o = Q @ origin[name]; c = Q @ com - o; I2 = Q @ I @ Q.T
    out.append(f'  <link name="{name}">')
    out.append(f'    <inertial><origin xyz="{fmt(c)}" rpy="0 0 0"/><mass value="{M:.6f}"/>')
    out.append(f'      <inertia ixx="{I2[0,0]:.8f}" ixy="{I2[0,1]:.8f}" ixz="{I2[0,2]:.8f}" iyy="{I2[1,1]:.8f}" iyz="{I2[1,2]:.8f}" izz="{I2[2,2]:.8f}"/></inertial>')
    vis, col = mesh_files[name]
    out.append(f'    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{vis}"/></geometry>'
               f'<material name="{name}"><color rgba="{COLORS[name]} 1"/></material></visual>')
    out.append(f'    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{col}"/></geometry></collision>')
    out.append('  </link>')
for j, par, ch, mate, key, lim in JOINTS:
    rel = Q @ (pivot[j] - origin[par]); ax = Q @ axis[j]
    out.append(f'  <joint name="{j}" type="revolute"><parent link="{par}"/><child link="{ch}"/>'
               f'<origin xyz="{fmt(rel)}" rpy="0 0 0"/><axis xyz="{fmt(ax)}"/>'
               f'<limit lower="{math.radians(lim[0]):.6f}" upper="{math.radians(lim[1]):.6f}" effort="{EFFORT}" velocity="{VEL}"/></joint>')
out.append('</robot>')
open(HERE.parent / "lamp.urdf", "w").write("\n".join(out) + "\n")
for n, (M, com, I) in link.items(): print(f"{n:10s} {M*1000:7.1f} g  eig(I)={np.round(np.linalg.eigvalsh(I)*1e6,2)} g*mm2/1000")
print("total", round(sum(v[0] for v in link.values()), 4), "kg")
for j in axis: print(j, "pivot(urdf)", np.round(Q @ pivot[j], 4), "axis", np.round(Q @ axis[j], 4))
