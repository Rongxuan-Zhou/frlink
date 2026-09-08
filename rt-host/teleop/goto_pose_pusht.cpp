// goto_pose_pusht.cpp — 把末端慢速移到 PushT 起始姿 home_pusht
// home_pusht = pos(0.665, -0.517, 0.098), R = 标准俯视 Rx(180°)
//   ("机械臂远端右侧标定点" x=0.665 远端 / y=-0.517 机械臂自身右 / z=0.098 PushT 高度)
//
// 设计：Cartesian pose 控制 + cosine ease（C1 连续，杜绝加速度突变 reflex），
//       位置 lerp + 姿态 quaternion slerp，速度按距离自适应并设上限，
//       起手 automaticErrorRecovery（teleop/自碰后常驻 Reflex）。
//
// 用法: goto_pose_pusht <ip> [--speed v(m/s, default 0.05)] [--z Z] [--x X] [--y Y]
//                          [--mode top|side]
//   top (default): 标准俯视 Rx(180°), EE z 轴朝下, 推杆垂直
//   side         : 侧推构型, EE z 轴朝 +x(向 T 块方向), 推杆水平向前
#include <franka/exception.h>
#include <franka/robot.h>
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include "examples_common.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: " << argv[0]
              << " <ip> [--speed v] [--x X] [--y Y] [--z Z] [--mode top|side]\n";
    return 1;
  }
  std::string ip = argv[1];
  double v_lin = 0.05;                 // m/s 平移上限
  double tx = 0.665, ty = -0.517, tz = 0.098;
  std::string mode = "top";
  for (int i = 2; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--speed" && i + 1 < argc) v_lin = std::stod(argv[++i]);
    else if (a == "--x" && i + 1 < argc) tx = std::stod(argv[++i]);
    else if (a == "--y" && i + 1 < argc) ty = std::stod(argv[++i]);
    else if (a == "--z" && i + 1 < argc) tz = std::stod(argv[++i]);
    else if (a == "--mode" && i + 1 < argc) mode = argv[++i];
  }

  // 目标姿态
  Eigen::Matrix3d R_goal;
  if (mode == "side") {
    // 侧推：EE z 轴朝 +x (推杆水平指向 T 块), EE y 轴朝 -y, EE x 轴朝 +z
    // R 列 = [x_in_world, y_in_world, z_in_world]
    R_goal << 0, 0, 1,
              0, -1, 0,
              1, 0, 0;
    std::cout << "[mode=side] 侧推构型: EE z 轴→+x, 推杆水平向前\n";
  } else {
    // top (default): 标准俯视 Rx(180°) → EE z 轴朝下
    R_goal << 1, 0, 0,
              0, -1, 0,
              0, 0, -1;
    std::cout << "[mode=top] 俯视构型: EE z 轴→-z, 推杆垂直\n";
  }
  Eigen::Quaterniond q_goal(R_goal);
  Eigen::Vector3d p_goal(tx, ty, tz);

  try {
    franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
    setDefaultBehavior(robot);
    robot.setCollisionBehavior(
        {{40, 40, 40, 40, 40, 40, 40}}, {{40, 40, 40, 40, 40, 40, 40}},
        {{40, 40, 40, 40, 40, 40}}, {{40, 40, 40, 40, 40, 40}});

    try {
      robot.automaticErrorRecovery();
      std::cout << "automaticErrorRecovery OK\n";
    } catch (const franka::Exception& e) {
      std::cerr << "⚠️ recovery: " << e.what() << " (continue)\n";
    }

    franka::RobotState s0 = robot.readOnce();
    Eigen::Vector3d p0_est(s0.O_T_EE[12], s0.O_T_EE[13], s0.O_T_EE[14]);
    double dist = (p_goal - p0_est).norm();
    double duration = std::max(dist / v_lin, 6.0);               // 至少 6s
    std::cout << "current pos = (" << p0_est.x() << ", " << p0_est.y() << ", "
              << p0_est.z() << "), target = (" << tx << ", " << ty << ", " << tz
              << ")\ndist=" << dist << " m, duration=" << duration
              << " s (quintic) → moving...\n";

    Eigen::Vector3d p0;
    Eigen::Quaterniond q0;
    bool inited = false;
    double time = 0.0;
    robot.control([&](const franka::RobotState& st,
                      franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();
      if (!inited) {                                             // 首拍：严格采机器人当前位姿
        Eigen::Affine3d T0(Eigen::Matrix4d::Map(st.O_T_EE.data()));
        p0 = T0.translation();
        q0 = Eigen::Quaterniond(T0.rotation());
        if (q0.dot(q_goal) < 0.0) q0.coeffs() = -q0.coeffs();    // 最短弧
        inited = true;
      }
      double u = std::min(time / duration, 1.0);
      // quintic: s(0)=0 s'(0)=0 s''(0)=0 → 速度+加速度均从 0 起，杜绝
      // cartesian_motion_generator_joint_acceleration_discontinuity
      double s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
      Eigen::Vector3d p = (1.0 - s) * p0 + s * p_goal;
      Eigen::Quaterniond q = q0.slerp(s, q_goal);
      Eigen::Affine3d T = Eigen::Affine3d::Identity();
      T.linear() = q.normalized().toRotationMatrix();
      T.translation() = p;
      std::array<double, 16> out{};
      Eigen::Matrix4d::Map(out.data()) = T.matrix();             // col-major
      if (time >= duration) {
        std::cout << "✅ at home_pusht\n";
        return franka::MotionFinished(franka::CartesianPose(out));
      }
      return franka::CartesianPose(out);
    });
    return 0;
  } catch (const franka::Exception& e) {
    std::cerr << "❌ " << e.what() << "\n";
    return 2;
  }
}
