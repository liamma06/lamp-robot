"""Static holding torque per joint (gravity only) over the whole joint range vs servo limit."""
import itertools
import numpy as np
import pybullet as p
from run_sim import load

STALL_NM = 2.94      # STS3215 at 12 V (~30 kg*cm); same as the URDF effort limit
RATED_NM = 0.98      # rated/continuous torque at 12 V (~10 kg*cm): can hold indefinitely
MARGIN = 0.5         # brief holds / passing through: up to 50% of stall

if __name__ == "__main__":
    robot, joints = load(gui=False)
    n = p.getNumJoints(robot)
    lims = [(p.getJointInfo(robot, j)[8], p.getJointInfo(robot, j)[9]) for j in joints]

    def torques(q):
        for j, v in zip(joints, q):
            p.resetJointState(robot, j, v)
        pos = [p.getJointState(robot, i)[0] for i in range(n)]
        return np.abs(p.calculateInverseDynamics(robot, pos, [0] * n, [0] * n))[joints]

    print("zero pose:", np.round(torques([0] * 6), 3), "N*m")
    # gravity torque depends mainly on pitch joints J2, J3, J5 and the roll J4; grid those, keep J1/J6 at 0
    grid = [np.linspace(lo, hi, 7) for lo, hi in lims]
    worst, where = np.zeros(6), [None] * 6
    for q2, q3, q4, q5 in itertools.product(grid[1], grid[2], grid[3], grid[4]):
        t = torques([0, q2, q3, q4, q5, 0])
        for i in range(6):
            if t[i] > worst[i]:
                worst[i], where[i] = t[i], (q2, q3, q4, q5)
    brief = STALL_NM * MARGIN
    print(f"\nworst-case holding torque per joint (continuous <= {RATED_NM} N*m, brief <= {brief:.2f} N*m):")
    for i, j in enumerate(joints):
        name = p.getJointInfo(robot, j)[1].decode()
        deg = np.degrees(where[i]).round(0) if where[i] else "-"
        print(f"  {name}  {worst[i]:.3f} N*m  {'OK' if worst[i] <= RATED_NM else ('BRIEF-ONLY' if worst[i] <= brief else 'OVER')}  at J2-J5 = {deg}")
