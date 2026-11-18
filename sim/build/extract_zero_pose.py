"""Run with main.SLDASM open in SolidWorks:  python build/extract_zero_pose.py
Read zero-pose geometry + joint directions from the open SolidWorks assembly.
Edits angle mates in memory only (never saves), checks every step, restores originals."""
import json, math, types
from pathlib import Path
import numpy as np
import win32com.client as wc

OUT = Path(__file__).with_name("zero_pose.json")
ZERO_DEG = {"J1 angle": 90.0, "J2 angle": 60.0, "J3 angle": 40.0, "J4 angle": 90.0, "J5 angle": 100.0}
PERTURB = 5.0
def call(o,n,*a):
    v=getattr(o,n); return v(*a) if isinstance(v,types.MethodType) else v
sw = wc.Dispatch("SldWorks.Application"); d = sw.ActiveDoc
def angle_dims():
    out={}
    f=call(d,'FirstFeature')
    while f:
        if call(f,'GetTypeName2')=="MateGroup":
            m=call(f,'GetFirstSubFeature')
            while m:
                nm=call(m,'Name')
                if nm.endswith(" angle"):
                    dd=call(m,'GetFirstDisplayDimension')
                    dim=call(dd,'GetDimension2',0)
                    out[nm]=dim
                m=call(m,'GetNextSubFeature')
        f=call(f,'GetNextFeature')
    return out

dims = angle_dims()
orig = {k: v.SystemValue for k, v in dims.items()}

def setdim(k, rad):
    dims[k].SetSystemValue3(rad, 1, None); d.ForceRebuild3(False)
def snap():
    out = {}
    for c in call(d, 'GetComponents', True):
        a = list(c.Transform2.ArrayData)
        out[c.Name2] = (np.array(a[0:9]).reshape(3, 3).T, np.array(a[9:12]))  # colmajor verified
    return out
def maxdiff(a, b):
    return max(max(np.abs(a[n][0]-b[n][0]).max(), np.abs(a[n][1]-b[n][1]).max()) for n in a)
def axes():
    out = {}; f = call(d, 'FirstFeature')
    while f:
        if call(f, 'GetTypeName2') == "MateGroup":
            m = call(f, 'GetFirstSubFeature')
            while m:
                nm = call(m, 'Name')
                if nm.startswith("Concentric"):
                    mate = call(m, 'GetSpecificFeature2'); ents = []
                    for i in range(call(mate, 'GetMateEntityCount')):
                        e = mate.MateEntity(i); p = list(e.EntityParams)
                        ents.append({"comp": e.ReferenceComponent.Name2 if e.ReferenceComponent else None, "pt": p[0:3], "dir": p[3:6]})
                    out[nm] = ents
                m = call(m, 'GetNextSubFeature')
        f = call(f, 'GetNextFeature')
    return out
def rotvec(R):
    ang = math.acos(max(-1, min(1, (np.trace(R)-1)/2)))
    if ang < 1e-9: return [0, 0, 0], 0.0
    ax = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])/(2*math.sin(ang))
    return ax.tolist(), math.degrees(ang)

res = {"abort": None}
try:
    base0 = snap()
    assert abs(math.degrees(orig["J2 angle"]) - 60) < 1e-6
    # 1. rebuild with no change: does the pose hold?
    d.ForceRebuild3(False); b1 = snap()
    res["rebuild_no_change_maxdiff"] = maxdiff(base0, b1)
    if res["rebuild_no_change_maxdiff"] > 1e-6: raise SystemExit("pose changed on plain rebuild")
    # 2. J1 -> 90
    setdim("J1 angle", math.radians(90)); z = snap()
    dyaw = math.radians(90) - orig["J1 angle"]
    Ry = np.array([[math.cos(dyaw),0,math.sin(dyaw)],[0,1,0],[-math.sin(dyaw),0,math.cos(dyaw)]])
    err = 0
    for n, (R, t) in z.items():
        R0, t0 = base0[n]
        moved = np.linalg.norm(t - t0) > 1e-9 or np.abs(R - R0).max() > 1e-9
        if moved:
            err = max(err, min(np.abs(t - Ry @ t0).max(), np.abs(t - Ry.T @ t0).max()))
    res["j1_snap_rigid_rotation_err"] = err
    if err > 1e-6: raise SystemExit("J1 snap moved parts non-rigidly (pose flipped)")
    res["axes_at_zero"] = axes()
    res["zero_tf"] = {n: {"R": R.tolist(), "t": t.tolist()} for n, (R, t) in z.items()}
    res["mass"] = {}
    for c in call(d, 'GetComponents', True):
        mp = c.GetModelDoc2.Extension.CreateMassProperty
        res["mass"][c.Name2] = {"mass": mp.Mass, "com_local": list(mp.CenterOfMass), "I_com_local": list(mp.GetMomentOfInertia(0))}
    # 3. perturb J2..J5 (and J1) by +5 deg
    res["perturb"] = {}
    for k in ZERO_DEG:
        setdim(k, math.radians(ZERO_DEG[k] + PERTURB)); s = snap(); mv = {}
        for n, (R, t) in s.items():
            R0, t0 = z[n]
            ax, ang = rotvec(R @ R0.T)
            if np.linalg.norm(t - t0) > 1e-7 or ang > 1e-5:
                mv[n] = {"axis": ax, "deg": ang}
        res["perturb"][k] = mv
        setdim(k, math.radians(ZERO_DEG[k])); back = snap()
        res.setdefault("return_err", {})[k] = maxdiff(z, back)
        if res["return_err"][k] > 1e-6: raise SystemExit(f"{k}: did not return to zero pose after perturb")
finally:
    for k, v in orig.items(): setdim(k, v)
    fin = snap()
    res["final_vs_original_maxdiff"] = maxdiff(base0, fin)
    json.dump(res, open(OUT, "w"), indent=1)
print({k: res.get(k) for k in ("rebuild_no_change_maxdiff", "j1_snap_rigid_rotation_err", "return_err", "final_vs_original_maxdiff")})
for k, m in res.get("perturb", {}).items(): print(k, {n.split("^")[0]: round(v["deg"], 2) for n, v in m.items()})
