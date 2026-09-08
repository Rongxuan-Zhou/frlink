"""franky_helpers.py — reusable franky safe-motion recipes validated in the 2026-06-08 session.

Hard-won lessons (details in the memory file franky-integration.md):
  1. On PREEMPT_DYNAMIC you must use RealtimeConfig.Ignore, otherwise the connection fails.
  2. ReferenceType.Relative displacements are in the *EE frame*; in the top-down pose EE-z points down → relative +z drives into the table.
     A world-frame lift must use an *absolute target* (see lift_to).
  3. Moving directly from a table-contact pose (EE z≈PLANAR_Z) drives into the table → cartesian_reflex
     (unrelated to collision thresholds; it is a real contact force). Run ensure_clearance to clear the table first.
  4. Before moving, replicate examples_common setDefaultBehavior (see setup).

Run with: ~/franka/.venv_franky/bin/python ...
"""
import math
import numpy as np
import franky
from franky import (Robot, JointMotion, CartesianMotion, Affine,
                    RealtimeConfig, Frame)

ROBOT_IP = "172.16.0.2"
PLANAR_Z = 0.098            # EE-to-table contact height
TABLE_CLEAR_Z = 0.15       # safe transit height (clear of the table)

# Standard rotation matrices (columns = EE axes expressed in the world frame)
R_TOP  = np.array([[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]])   # top-down Rx180, EE-z points down
R_SIDE = np.array([[0., 0., 1.], [0., -1., 0.], [1., 0., 0.]])    # sideways, EE-z points +x

# factory-ready home (= goto_home --mode home)
HOME_Q = [0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4]


def connect(ip=ROBOT_IP):
    """Connect to the robot (RealtimeConfig.Ignore on PREEMPT_DYNAMIC). FCI must be idle (servo off)."""
    return Robot(ip, realtime_config=RealtimeConfig.Ignore)


def setup(robot, load_mass=0.0, com=(0., 0., 0.05), collision=40, factor=0.04):
    """Replicate examples_common::setDefaultBehavior + payload + collision thresholds + slow speed.

    load_mass: 0.0 for a bare flange; pass the calibrated value when the Franka Hand / a tool is attached.
    collision: force/torque thresholds (N, Nm); goto_home uses 30, goto_pose uses 40.
    factor:    relative_dynamics_factor; 0.04 ≈ very slow and very safe.
    """
    if robot.has_errors:
        robot.recover_from_errors()
    robot.set_joint_impedance([3000, 3000, 3000, 2500, 2500, 2000, 2000])
    robot.set_cartesian_impedance([3000, 3000, 3000, 300, 300, 300])
    robot.set_load(load_mass, list(com), [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4])
    robot.set_collision_behavior([collision]*7, [collision]*6)
    robot.relative_dynamics_factor = factor


def affine_from(R, p):
    """Build a franky.Affine from rotation matrix R (3x3) + position p (3) (via the 4x4 matrix constructor, no hand-rolled quaternions)."""
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float)
    T[:3, 3] = np.asarray(p, float)
    return Affine(T)


def current_ee(robot):
    """Return the current end-effector pose as (translation[3], quaternion[xyzw][4])."""
    a = robot.current_cartesian_state.pose.end_effector_pose
    return np.array(a.translation), np.array(a.quaternion)


def fk_z(robot, q):
    """Forward kinematics: world z height of the EE at joint configuration q (used for the table-sweep path pre-check)."""
    st = robot.state
    T = robot.model.pose(Frame.EndEffector, np.asarray(q, float).reshape(7, 1),
                         st.F_T_EE, st.EE_T_K)
    return float(T.translation[2])


def path_min_z(robot, q0, q1, n=41):
    """Minimum EE z along the linear joint-interpolation path (predicts whether it would sweep the table)."""
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
    return min(fk_z(robot, q0 + t*(q1 - q0)) for t in np.linspace(0, 1, n))


def lift_to(robot, z_target, factor=0.04):
    """World-frame *absolute* lift of the EE to z_target (orientation unchanged). Returns whether a lift actually happened.

    ⚠️ Do not use Relative([0,0,dz]): in the top-down pose that drives into the table (see module note 2).
    """
    t, q = current_ee(robot)
    if t[2] >= z_target - 1e-4:
        return False
    robot.relative_dynamics_factor = factor
    robot.move(CartesianMotion(Affine([t[0], t[1], z_target], q)))
    return True


def ensure_clearance(robot, z=TABLE_CLEAR_Z, factor=0.04):
    """If the EE is currently at the table (z < clearance), lift to clear it first. Returns whether a lift happened."""
    return lift_to(robot, z, factor)


def goto_home(robot, factor=0.04):
    """Safely return to the factory-ready home: clear the table → FK path pre-check → slow joint move to home."""
    ensure_clearance(robot, factor=factor)
    q0 = robot.current_joint_state.position
    zmin = path_min_z(robot, q0, HOME_Q)
    if zmin < 0.105:
        raise RuntimeError(f"Lowest z on the path to home is {zmin:.3f}m, still at table level; inspect manually before moving")
    robot.relative_dynamics_factor = factor
    robot.move(JointMotion(HOME_Q))
    return robot.current_joint_state.position


def goto_pose(robot, R, p, factor=0.04, lift_first=True):
    """Move to the target pose (R rotation matrix, p position). The endpoint may be on the table (z=PLANAR_Z);
    Ruckig smooth approach (replaces the C++ quintic+slerp).

    lift_first: if currently at the table, lift to clear it first, so the end effector is not dragged across the table.
    """
    if lift_first:
        ensure_clearance(robot, factor=factor)
    robot.relative_dynamics_factor = factor
    robot.move(CartesianMotion(affine_from(R, p)))
    return current_ee(robot)
