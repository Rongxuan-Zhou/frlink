// goto_home.cpp — Send the Franka to a target joint configuration (direct joint-space motion)
//
// Usage:
//   goto_home <ip> [--speed 0.1] [--mode home]
//   goto_home <ip> [--speed 0.1] --q q1 q2 q3 q4 q5 q6 q7  (rad, specify the 7 joint angles directly)
//
// Presets:
//   home       : Franka factory ready [0,-π/4,0,-3π/4,0,π/2,π/4]
//
// ⚠️ Joint-space interpolation does no collision checking; confirm the workspace is clear before moving, hand on the e-stop.
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
              << " <ip> [--speed 0.1] [--mode home]\n"
              << "       " << argv[0]
              << " <ip> [--speed 0.1] --q q1 q2 q3 q4 q5 q6 q7\n";
    return 1;
  }
  std::string ip = argv[1];
  double speed = 0.10;   // joint-space motion defaults to slower, safety first
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

  // Pick the target joint angles
  std::array<double, 7> q_target;
  std::string label;
  if (use_custom_q) {
    q_target = q_custom;
    label = "custom";
  } else if (mode != "home") {
    std::cerr << "unknown --mode '" << mode << "' (expected home)\n";
    return 1;
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

    MotionGenerator motion_generator(speed, q_target);
    robot.control(motion_generator);
    std::cout << " ✅ at [" << label << "]" << std::endl;
    return 0;
  } catch (const franka::Exception& e) {
    std::cerr << "❌ " << e.what() << std::endl;
    return 2;
  }
}
