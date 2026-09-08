"""franky_helpers.py — 复用本会话(2026-06-08)验证的 franky 安全运动配方。

沉淀的血泪经验(详见记忆 franky-integration.md):
  1. PREEMPT_DYNAMIC 必须用 RealtimeConfig.Ignore 才能连。
  2. ReferenceType.Relative 位移在 *EE 帧*;俯视姿 EE-z 朝下 → 相对 +z 是往下扎桌。
     世界帧抬升必须用 *绝对目标*(见 lift_to)。
  3. 从贴桌位姿(推杆 z≈PLANAR_Z)直接运动会扎桌 → cartesian_reflex
     (与碰撞阈值无关,是真接触力)。先 ensure_clearance 清桌再动。
  4. 运动前复刻 examples_common setDefaultBehavior(见 setup)。

运行: ~/franka/.venv_franky/bin/python ...
"""
import math
import numpy as np
import franky
from franky import (Robot, JointMotion, CartesianMotion, Affine,
                    RealtimeConfig, Frame)

ROBOT_IP = "172.16.0.2"
PLANAR_Z = 0.098            # 推杆接触桌面高度
TABLE_CLEAR_Z = 0.15       # 安全过渡高度(清桌)

# 标准旋转矩阵(列 = EE 各轴在世界系的方向),来自 goto_pose_pusht.cpp
R_TOP  = np.array([[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]])   # 俯视 Rx180, EE-z 朝下, 推杆垂直
R_SIDE = np.array([[0., 0., 1.], [0., -1., 0.], [1., 0., 0.]])    # 侧推, EE-z 朝 +x, 推杆水平

# factory-ready home(= goto_home --mode home)
HOME_Q = [0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4]


def connect(ip=ROBOT_IP):
    """连接机器人(PREEMPT_DYNAMIC 用 RealtimeConfig.Ignore)。FCI 须空闲(servo 关)。"""
    return Robot(ip, realtime_config=RealtimeConfig.Ignore)


def setup(robot, load_mass=0.0, com=(0., 0., 0.05), collision=40, factor=0.04):
    """复刻 examples_common::setDefaultBehavior + 载荷 + 碰撞阈值 + 慢速。

    load_mass: PushT 无夹爪用 0.0;带 UMI/夹爪可传标定值。
    collision: 力/力矩阈值(N, Nm),goto_home 用 30 / goto_pose_pusht 用 40。
    factor:    relative_dynamics_factor,0.04 ≈ 很慢很安全。
    """
    if robot.has_errors:
        robot.recover_from_errors()
    robot.set_joint_impedance([3000, 3000, 3000, 2500, 2500, 2000, 2000])
    robot.set_cartesian_impedance([3000, 3000, 3000, 300, 300, 300])
    robot.set_load(load_mass, list(com), [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4])
    robot.set_collision_behavior([collision]*7, [collision]*6)
    robot.relative_dynamics_factor = factor


def affine_from(R, p):
    """由旋转矩阵 R(3x3) + 位置 p(3) 造 franky.Affine(走 4x4 矩阵构造,免手搓四元数)。"""
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float)
    T[:3, 3] = np.asarray(p, float)
    return Affine(T)


def current_ee(robot):
    """返回 (translation[3], quaternion[xyzw][4]) 当前末端位姿。"""
    a = robot.current_cartesian_state.pose.end_effector_pose
    return np.array(a.translation), np.array(a.quaternion)


def fk_z(robot, q):
    """正运动学算关节配置 q 的 EE 世界 z 高度(用于路径扫桌预检)。"""
    st = robot.state
    T = robot.model.pose(Frame.EndEffector, np.asarray(q, float).reshape(7, 1),
                         st.F_T_EE, st.EE_T_K)
    return float(T.translation[2])


def path_min_z(robot, q0, q1, n=41):
    """关节直线插值路径上 EE 的最低 z(预判是否扫桌)。"""
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
    return min(fk_z(robot, q0 + t*(q1 - q0)) for t in np.linspace(0, 1, n))


def lift_to(robot, z_target, factor=0.04):
    """世界帧*绝对*抬升 EE 到 z_target(姿态不变)。返回是否实际抬升。

    ⚠️ 不能用 Relative([0,0,dz]):俯视姿那是往下扎桌(见模块注释 2)。
    """
    t, q = current_ee(robot)
    if t[2] >= z_target - 1e-4:
        return False
    robot.relative_dynamics_factor = factor
    robot.move(CartesianMotion(Affine([t[0], t[1], z_target], q)))
    return True


def ensure_clearance(robot, z=TABLE_CLEAR_Z, factor=0.04):
    """若当前 EE 贴桌(z < clearance),先抬升清桌。返回是否抬升。"""
    return lift_to(robot, z, factor)


def goto_home(robot, factor=0.04):
    """安全回 factory-ready home:先清桌 → FK 路径预检 → 关节慢速回 home。"""
    ensure_clearance(robot, factor=factor)
    q0 = robot.current_joint_state.position
    zmin = path_min_z(robot, q0, HOME_Q)
    if zmin < 0.105:
        raise RuntimeError(f"回 home 路径最低 z={zmin:.3f}m 仍贴桌,人工检查再动")
    robot.relative_dynamics_factor = factor
    robot.move(JointMotion(HOME_Q))
    return robot.current_joint_state.position


def goto_pose(robot, R, p, factor=0.04, lift_first=True):
    """移到目标位姿(R 旋转矩阵, p 位置)。终点可在桌面(PushT z=0.098),
    Ruckig 平滑趋近(替代 C++ 的 quintic+slerp)。

    lift_first: 当前若贴桌先抬升清桌,避免横扫桌面拖拽推杆。
    """
    if lift_first:
        ensure_clearance(robot, factor=factor)
    robot.relative_dynamics_factor = factor
    robot.move(CartesianMotion(affine_from(R, p)))
    return current_ee(robot)
