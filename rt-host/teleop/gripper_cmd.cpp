// gripper_cmd.cpp — One-shot gripper command (open/close/grasp)
// Usage:
//   gripper_cmd <ip> open                       # open to max
//   gripper_cmd <ip> close                      # close to 0
//   gripper_cmd <ip> width <w_m> [--speed 0.1]  # move to the specified width (meters)
//   gripper_cmd <ip> grasp <w_m> [--speed 0.1] [--force 30]
//   gripper_cmd <ip> homing                     # calibrate (~10s)
//   gripper_cmd <ip> read                       # print state
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
  double force = 60.0;   // 30 → 60 N continuous force (Franka Hand max 70 N continuous)
                          // UMI 3D compliant fingers need sustained force to keep the deformed fit
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
      // Use grasp() for sustained force, not move(0)
      // move() is position control: it stops on contact with the object but does not keep pressing, so the UMI soft fingers spring back
      // grasp(target=0, speed, force, eps_inner, eps_outer) keeps applying force to maintain the grip
      // eps_inner/outer set large (max_width) so any final width counts as success → sustained force
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
