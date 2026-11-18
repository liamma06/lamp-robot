# Lamp sim (PyBullet)

## Setup
```
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```
Run everything from this `sim/` folder.

## Use it
- `python view_sim.py`: interactive 3D window with a slider per joint, live torque and collision warnings
- `python run_sim.py`: sweeps each joint through its limits, reports self-collisions
- `python torque_check.py`: worst-case holding torque per joint vs the STS3215 (12 V) limits

`lamp.urdf` is the robot description the three scripts load.

## Meshes
`meshes/` holds one STL per CAD part (exported from SolidWorks in assembly coordinates, metres): the main parts in `meshes/`, the servo internals in `meshes/servos/`. `build/gen_urdf.py` merges them per link into `meshes/links/` (visual mesh + convex-hull collision shape), which `lamp.urdf` loads. Re-export the STLs from SolidWorks whenever the CAD changes, then re-run `gen_urdf.py`.

## Rebuilding the model from CAD (`build/`)
1. Open `3D_assets/main.SLDASM` in SolidWorks (Windows, SolidWorks running).
2. `python build/extract_zero_pose.py` reads the assembly through the SolidWorks API and writes `build/zero_pose.json`. It temporarily edits angle mates in memory and restores them; it never saves.
3. `python build/gen_urdf.py` turns that JSON and the STLs into `lamp.urdf`. Servo mass, effort, joint limits and link grouping are set at the top of this file.

Zero pose = the CAD pose with mate angles J1 90, J2 60, J3 40, J4 90, J5 100 (J6 has no mate). Positive directions were measured by perturbing each mate; J6's is an assumption.
