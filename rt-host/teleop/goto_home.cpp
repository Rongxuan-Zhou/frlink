// goto_home.cpp — 把 Franka 送到目标关节配置（关节空间直接运动）
//
// 用法:
//   goto_home <ip> [--speed 0.1] [--mode home|pusht-side]
//   goto_home <ip> [--speed 0.1] --q q1 q2 q3 q4 q5 q6 q7  (rad, 直接指定7个关节角)
//
// 预设:
//   home       : Franka factory ready [0,-π/4,0,-3π/4,0,π/2,π/4]
//   pusht-side : DAIRLab push_t 侧推构型 [2.191,1.1,-1.33,-2.22,1.30,2.02,0.08]
//                J1=125°(大幅侧向), J2=63°(肩抬高), J6=116°(腕居中)
//
// ⚠️ 关节空间插值不做碰撞检测，移动前确认工作区清空，手放急停。
#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <ip> [--speed 0.1] [--mode home|pusht-side]\n"
              << "       " << argv[0]
              << " <ip> [--speed 0.1] --q q1 q2 q3 q4 q5 q6 q7\n";
    return 1;
  }
  std::string ip = argv[1];
  double speed = 0.10;   // 关节空间运动默认更慢，安全第一
  std::string mode = "home";
  std::array<double, 7> q_custom{};
  bool use_custom_q = false;

  for (int i = 2; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--speed" && i + 1 < argc) { speed = std::stod(argv[++i]); }
    else if (a == "--mode" && i + 1 < argc) { mode = argv[++i]; }
    else if (a == "--q" && i + 7 < argc) {
      for (int j = 0; j < 7; ++j) q_custom[j] = std::stod(argv[++i]);
      use_custom_q = true;
    }
  }
  setvbuf(stdout, nullptr, _IOLBF, 0);

  // 选目标关节角
  std::array<double, 7> q_target;
  std::string label;
  if (use_custom_q) {
    q_target = q_custom;
    label = "custom";
  } else if (mode == "pusht-side") {
    // DAIRLab push_t sim_params.yaml q_init_franka
    // J1=125.5°(大幅侧向), J2=63°(肩抬高), J5=74.5°(前臂旋), J6=116°(腕居中)
    q_target = {{2.191, 1.1, -1.33, -2.22, 1.30, 2.02, 0.08}};
    label = "pusht-side (DAIRLab)";
  } else {
    q_target = {{0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4}};
    label = "factory ready";
  }

  std::cout << "target [" << label << "]: ";
  for (int i = 0; i < 7; ++i)
    std::cout << q_target[i] << (i < 6 ? " " : "\n");

  try {
    franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
    setDefaultBehavior(robot);
    robot.setLoad(0.0, {0.0, 0.0, 0.05}, {1e-4,0,0, 0,1e-4,0, 0,0,1e-4});
    robot.setCollisionBehavior({{30,30,30,30,30,30,30}}, {{30,30,30,30,30,30,30}},
                               {{30,30,30,30,30,30}},   {{30,30,30,30,30,30}});
    try {
      robot.automaticErrorRecovery();
      std::cout << "automaticErrorRecovery OK" << std::endl;
    } catch (const franka::Exception& e) {
      std::cerr << "⚠️ recovery: " << e.what() << " (continue)" << std::endl;
    }

    auto state = robot.readOnce();
    std::cout << "current q: ";
    for (auto v : state.q) std::cout << v << " ";
    std::cout << "\n→ moving to [" << label << "] speed=" << speed << " ..." << std::endl;

    // pusht-side 需要三段路点：直接插值会让 EE 扫出 Cartesian 安全包络
    //   via1: 手臂竖起（EE 近机器人上方，J1 旋转时占地小）
    //   via2: J1 旋到目标角度（手臂仍竖起）
    //   final: 展开到 pusht-side 目标
    if (mode == "pusht-side" && !use_custom_q) {
      // via1: 肩关节抬高(J2正值)，肘深度弯曲，手臂紧凑收在上方
      //       J2=+46° 上臂朝上, J4=-160° 前臂折回, EE 在机器人上方附近
      //       ⚠️ 不能用 J2 负值（往下压会戳桌面）
      std::array<double, 7> q_via1 = {{0.0, 0.8, 0.0, -2.8, 0.0, 2.0, 0.785}};
      std::cout << "  → via1 (arm up: J2=+46°, J4=-160°, EE near top) ..." << std::flush;
      MotionGenerator mg1(speed, q_via1);
      robot.control(mg1);
      std::cout << " ✅\n";

      // via2: J1 旋到目标，手臂保持竖起（EE 在上方小圆弧，不出 Cartesian 安全包络）
      std::array<double, 7> q_via2 = {{2.191, 0.8, 0.0, -2.8, 0.0, 2.0, 0.785}};
      std::cout << "  → via2 (J1→125°, arm still up) ..." << std::flush;
      MotionGenerator mg2(speed * 0.6, q_via2);
      robot.control(mg2);
      std::cout << " ✅\n";

      // final: 展开到 pusht-side 目标
      std::cout << "  → final (pusht-side) ..." << std::flush;
    }
    MotionGenerator motion_generator(speed, q_target);
    robot.control(motion_generator);
    std::cout << " ✅ at [" << label << "]" << std::endl;
    return 0;
  } catch (const franka::Exception& e) {
    std::cerr << "❌ " << e.what() << std::endl;
    return 2;
  }
}
