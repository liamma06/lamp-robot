"""Interactive viewer: one slider per joint (degrees from the CAD zero pose).
Shows live holding torque per joint and warns on self-collision.
Mouse: left-drag rotate, right-drag/scroll zoom, ctrl+left-drag pan. Close the window to quit."""
import time
import pybullet as p
from run_sim import load, self_collisions

STALL_NM, RATED_NM = 2.94, 0.98

robot, joints = load(gui=True)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.resetDebugVisualizerCamera(cameraDistance=0.7, cameraYaw=60, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.15])
n = p.getNumJoints(robot)
names = {i: p.getJointInfo(robot, i)[12].decode() for i in range(n)}   # link names
sliders = {}
for j in joints:
    info = p.getJointInfo(robot, j)
    lo, hi = round(info[8] * 57.2958), round(info[9] * 57.2958)
    sliders[j] = p.addUserDebugParameter(f"{info[1].decode()} ({names[j]}) deg", lo, hi, 0)
reset = p.addUserDebugParameter("RESET (click to zero all)", 1, 0, 0)  # button: value toggles on click
last_reset = p.readUserDebugParameter(reset)
status = p.addUserDebugText("", [0, 0, 0.45], textSize=1.2, textColorRGB=[1, 1, 1])

while p.isConnected():
    if p.readUserDebugParameter(reset) != last_reset:
        last_reset = p.readUserDebugParameter(reset)
        for j in joints:
            p.resetDebugVisualizerCamera(0.7, 60, -20, [0, 0, 0.15])
        # sliders can't be set from code, so tell the user
        print("Reset clicked: set the sliders back to 0 by hand (PyBullet can't move sliders from code).")
    q = [p.readUserDebugParameter(sliders[j]) * 0.0174533 for j in joints]
    for j, v in zip(joints, q):
        p.resetJointState(robot, j, v)
    p.performCollisionDetection()
    hits = self_collisions(robot)
    pos = [p.getJointState(robot, i)[0] for i in range(n)]
    tau = p.calculateInverseDynamics(robot, pos, [0] * n, [0] * n)
    lines = []
    for j in joints:
        t = abs(tau[j]); tag = "ok" if t <= RATED_NM else ("brief" if t <= STALL_NM / 2 else "OVER")
        lines.append(f"{p.getJointInfo(robot, j)[1].decode()} {t:.2f}Nm {tag}")
    msg = "  ".join(lines) + ("   COLLISION " + str(sorted(names.get(a, 'base') + '-' + names.get(b, 'base') for a, b in hits)) if hits else "")
    status = p.addUserDebugText(msg, [0, 0, 0.45], textSize=1.1, textColorRGB=[1, 0.3, 0.3] if hits else [1, 1, 1], replaceItemUniqueId=status)
    time.sleep(1 / 60)
