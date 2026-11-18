"""Load the lamp, sweep each joint through its limits, report self-collisions.
Usage: python run_sim.py        (headless)   |   python run_sim.py --gui"""
import sys, time
import pybullet as p
import pybullet_data

GUI = "--gui" in sys.argv


def load(gui=None):
    p.connect(p.GUI if (GUI if gui is None else gui) else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("lamp.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
    joints = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE]
    return robot, joints


def self_collisions(robot):
    hits = set()
    for c in p.getContactPoints(robot, robot):
        a, b = c[3], c[4]
        if abs(a - b) > 1:          # adjacent links always touch at the joint
            hits.add((min(a, b), max(a, b)))
    return hits


def sweep(robot, joints, steps=60):
    for j in joints:
        info = p.getJointInfo(robot, j)
        name, lo, hi = info[1].decode(), info[8], info[9]
        bad = 0
        for k in range(steps + 1):
            p.resetJointState(robot, j, lo + (hi - lo) * k / steps)
            p.performCollisionDetection()
            bad += bool(self_collisions(robot))
            if GUI:
                time.sleep(0.01)
        p.resetJointState(robot, j, 0)
        print(f"{name:4s} [{lo:+.2f}, {hi:+.2f}] rad  self-collision poses: {bad}")


if __name__ == "__main__":
    robot, joints = load()
    p.resetJointState(robot, joints[0], 0)
    p.performCollisionDetection()
    print("zero pose self-collisions:", sorted(self_collisions(robot)) or "none")
    sweep(robot, joints)
    if GUI:
        time.sleep(2)
