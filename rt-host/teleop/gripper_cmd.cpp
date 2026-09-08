// gripper_cmd.cpp — 一次性夹爪命令（open/close/grasp）
// 用法:
//   gripper_cmd <ip> open                       # 张到 max
//   gripper_cmd <ip> close                      # 闭到 0
//   gripper_cmd <ip> width <w_m> [--speed 0.1]  # 移动到指定宽度 (米)
//   gripper_cmd <ip> grasp <w_m> [--speed 0.1] [--force 30]
//   gripper_cmd <ip> homing                     # 校准 (~10s)
//   gripper_cmd <ip> read                       # 打 state
#include <franka/exception.h>
#include <franka/gripper.h>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <ip> open|close|width <w>|grasp <w>|homing|read [--speed v] [--force f]\n";
    return 1;
  }
  std::string ip = argv[1];
  std::string cmd = argv[2];
  double speed = 0.1;
  double force = 60.0;   // 30 → 60 N 持续力 (Franka Hand max 70 N continuous)
                          // UMI 3D 柔性指需要持续施力维持形变贴合
  for (int i = 3; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--speed" && i + 1 < argc) speed = std::stod(argv[++i]);
    else if (a == "--force" && i + 1 < argc) force = std::stod(argv[++i]);
  }
  try {
    franka::Gripper g(ip);
    auto state = g.readOnce();
    if (cmd == "read") {
      std::cout << "{\"width\":" << state.width << ",\"max_width\":" << state.max_width
                << ",\"is_grasped\":" << (state.is_grasped ? "true" : "false")
                << ",\"temperature\":" << state.temperature << "}\n";
      return 0;
    }
    if (cmd == "open") {
      g.move(state.max_width, speed);
    } else if (cmd == "close") {
      // 用 grasp() 持续施力, 不用 move(0)
      // move() 是位置控制: 碰到物体停下但不持续顶, UMI 软指会回弹
      // grasp(target=0, speed, force, eps_inner, eps_outer) 持续施力维持夹紧
      // eps_inner/outer 设大 (max_width) 让任意 final width 都视为成功 → 持续 force
      g.grasp(0.0, speed, force, state.max_width, state.max_width);
    } else if (cmd == "homing") {
      g.homing();
    } else if (cmd == "width" || cmd == "grasp") {
      if (argc < 4) { std::cerr << "missing width\n"; return 2; }
      double w = std::stod(argv[3]);
      if (cmd == "width") {
        g.move(w, speed);
      } else {
        g.grasp(w, speed, force);
      }
    } else {
      std::cerr << "unknown cmd: " << cmd << "\n";
      return 3;
    }
    return 0;
  } catch (const franka::Exception& e) {
    std::cerr << "franka::Exception: " << e.what() << "\n";
    return 4;
  }
}
