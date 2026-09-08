// fk_probe.cpp — Path checker: given a target q (7 values), sample the linear joint interpolation from the current q,
// print the EE position + z at each step, and report the lowest z along the whole path (judges whether the end effector would hit the table). Read-only, does not move.
// Usage: fk_probe <ip> q1 q2 q3 q4 q5 q6 q7   (check the path current→that q)
//        fk_probe <ip>                          (only print the current FK)
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc < 2) { std::cerr << "usage: " << argv[0] << " <ip> [q1..q7]\n"; return 1; }
  try {
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
    franka::Model model = robot.loadModel();
    franka::RobotState s = robot.readOnce();
    std::array<double,7> q0 = s.q;

    auto fk = [&](const std::array<double,7>& q){
      std::array<double,16> T = model.pose(franka::Frame::kEndEffector, q, s.F_T_EE, s.EE_T_K);
      Eigen::Map<Eigen::Matrix4d> M(T.data());
      return std::make_pair(Eigen::Vector3d(M.block<3,1>(0,3)), Eigen::Vector3d(M.block<3,1>(0,2)));
    };

    std::cout.precision(5);
    auto [p0, z0] = fk(q0);
    std::cout << "current: EE=(" << p0.x() << "," << p0.y() << "," << p0.z() << ")\n";
    std::cout << "current q = ";
    for (int i = 0; i < 7; ++i) std::cout << q0[i] << (i < 6 ? " " : "\n");
    // Manipulability w = sqrt(det(J J^T)), judges whether near-singular (home must be non-singular)
    {
      std::array<double,42> J = model.zeroJacobian(franka::Frame::kEndEffector, q0,
                                                   s.F_T_EE, s.EE_T_K);
      Eigen::Map<Eigen::Matrix<double,6,7>> Jm(J.data());
      double w = std::sqrt((Jm * Jm.transpose()).determinant());
      std::cout << "manipulability w = " << w << (w > 0.03 ? " (non-singular✅)" : " (near-singular⚠️)") << "\n";
    }

    if (argc < 9) return 0;   // no target q given, only print current
    std::array<double,7> qt;
    for (int i = 0; i < 7; ++i) qt[i] = std::stod(argv[2 + i]);

    double zmin = 1e9; double zmin_t = 0;
    std::cout << "=== Path current→target linear interpolation samples (rod tip z)===\n";
    for (int k = 0; k <= 10; ++k) {
      double t = k / 10.0;
      std::array<double,7> q;
      for (int i = 0; i < 7; ++i) q[i] = (1 - t) * q0[i] + t * qt[i];
      auto [p, zax] = fk(q);
      std::cout << "  t=" << t << "  EE=(" << p.x() << "," << p.y() << "," << p.z()
                << ")  z-axis=(" << zax.x() << "," << zax.y() << "," << zax.z() << ")\n";
      if (p.z() < zmin) { zmin = p.z(); zmin_t = t; }
    }
    std::cout << ">>> lowest tip z along the path = " << zmin << " @t=" << zmin_t
              << (zmin > 0.08 ? "  ✅ does not poke the table (>0.08)" : "  🔴 would poke the table!(<0.08)") << "\n";
    return 0;
  } catch (const franka::Exception& e) {
    std::cerr << "❌ " << e.what() << "\n"; return 2;
  }
}
