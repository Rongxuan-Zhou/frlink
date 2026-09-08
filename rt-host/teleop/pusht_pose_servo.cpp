// pusht_pose_servo.cpp — PushT 专用 1kHz Cartesian 位置控制 servo
//
// 为什么独立于 cartesian_pose_servo.cpp（柔顺力矩阻抗）：
//   PushT = 2-DOF 平面任务（x,y 跟手 + 固定 z + 固定平姿）。需要刚性位置控制，
//   不是柔顺力矩阻抗。阻抗 servo 在伸展低位（model bias ~10N）+ 刚性姿态锁
//   + 7-DOF 零空间 → 慢发散飞天（实测 z=0.21~1.25），且柔顺本质上 hold 不住
//   严格平行/恒 z。位置控制（libfranka 内部控制器 robust 处理 model bias，
//   goto_pose_pusht/wiggle 已证明绝不飞）天生满足严格 z/姿态 + 稳定。
//   零风险：常规 teleop 仍用 cartesian_pose_servo，本文件不碰它。
//
// 链路：03_webxr_to_franka_pusht.py --UDP:50001--> 本 servo --CartesianPose--> FR3
//   python 端已锁 z=0.098 + 姿态俯视 + x,y 映射，本 servo 只做平滑跟随。
//
// 平滑：python ~90Hz 目标 → 1kHz 一阶 LPF 插值 + 每 tick 位移钳位
//   （杜绝 cartesian_motion_generator_velocity/acceleration_discontinuity）。
//
// 用法: pusht_pose_servo <ip> [--port P] [--alpha A] [--vmax V] [--load-mass M]
#include <franka/exception.h>
#include <franka/robot.h>
#include <Eigen/Dense>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include "examples_common.h"

namespace {
constexpr int kPoseBytes = 16 * sizeof(double);

std::atomic<bool> g_stop{false};
std::atomic<int64_t> g_last_recv_ms{0};

struct AtomicPose {
  std::array<double, 16> data{};
  std::mutex mu;
  bool valid{false};
  void set(const std::array<double, 16>& s) {
    std::lock_guard<std::mutex> lk(mu); data = s; valid = true;
  }
  bool get(std::array<double, 16>& out) {
    std::lock_guard<std::mutex> lk(mu); out = data; return valid;
  }
};

void on_sigint(int) { g_stop.store(true); }

int64_t now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

void udp_listener(int port, AtomicPose* target) {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) { std::cerr << "udp socket failed\n"; return; }
  int one = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(port);
  if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::cerr << "udp bind failed on " << port << "\n"; close(sock); return;
  }
  timeval tv{0, 100 * 1000};
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  std::array<double, 16> buf{};
  while (!g_stop.load()) {
    ssize_t n = recv(sock, buf.data(), kPoseBytes, 0);
    if (n == kPoseBytes) {
      if (std::isnan(buf[0])) continue;       // 忽略 magic 包（本 servo 无 home 功能）
      target->set(buf);
      g_last_recv_ms.store(now_ms());
    }
  }
  close(sock);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <ip> [--port P] [--alpha A] [--vmax V] [--load-mass M]\n";
    return 1;
  }
  std::string ip = argv[1];
  int port = 50001;
  double alpha = 0.05;       // 一阶 LPF 系数（90Hz 目标 → 1kHz 平滑跟随）
  double vmax = 0.30;        // m/s 每 tick 位移钳位上限（安全 + 防 vel 突变 reflex）
  double load_mass = 0.0;    // PushT 无夹爪+stick 实测最优 0.0（calibrate_payload）
  double load_com_z = 0.05;
  for (int i = 2; i + 1 < argc; i += 2) {
    std::string f = argv[i];
    if (f == "--port") port = std::stoi(argv[i + 1]);
    else if (f == "--alpha") alpha = std::stod(argv[i + 1]);
    else if (f == "--vmax") vmax = std::stod(argv[i + 1]);
    else if (f == "--load-mass") load_mass = std::stod(argv[i + 1]);
    else if (f == "--load-com-z") load_com_z = std::stod(argv[i + 1]);
  }
  const double max_step = vmax / 1000.0;   // 每 1kHz tick 最大位移

  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  AtomicPose target;
  std::thread udp_thr(udp_listener, port, &target);

  try {
    franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
    setDefaultBehavior(robot);
    robot.setCollisionBehavior(
        {{40, 40, 40, 40, 40, 40, 40}}, {{40, 40, 40, 40, 40, 40, 40}},
        {{40, 40, 40, 40, 40, 40}}, {{40, 40, 40, 40, 40, 40}});
    std::array<double, 9> kInertia{1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4};
    robot.setLoad(load_mass, {0.0, 0.0, load_com_z}, kInertia);
    std::cerr << "  setLoad m=" << load_mass << " kg (PushT pos-ctrl)\n";

    // init pose → 文件（python 读它做 mirror/init 基准）
    franka::RobotState s0 = robot.readOnce();
    {
      std::ofstream f("/tmp/franka_init_pose.txt");
      for (double v : s0.O_T_EE) f << v << " ";
      std::cerr << "  init pose saved to /tmp/franka_init_pose.txt\n";
    }

    // current EE 写线程（python dead-man 读它 → 原地 hold）
    AtomicPose cur_ee;
    cur_ee.set(s0.O_T_EE);
    std::thread ee_writer([&cur_ee]() {
      while (!g_stop.load()) {
        std::array<double, 16> e;
        if (cur_ee.get(e)) {
          std::ofstream f("/tmp/franka_current_ee.txt");
          for (double v : e) f << v << " ";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    });

    const int max_retries = 50;
    int retries = 0;
    while (retries < max_retries && !g_stop.load()) {
      try {
        try { robot.automaticErrorRecovery(); } catch (...) {}

        bool inited = false;
        Eigen::Vector3d p_cmd;
        Eigen::Quaterniond q_cmd;

        std::cerr << "→ PushT pos-ctrl 1kHz. Ctrl-C to stop.\n";
        robot.control([&](const franka::RobotState& st,
                          franka::Duration) -> franka::CartesianPose {
          cur_ee.set(st.O_T_EE);
          if (!inited) {                       // 首拍：commanded = 机器人当前位姿
            Eigen::Affine3d T0(Eigen::Matrix4d::Map(st.O_T_EE.data()));
            p_cmd = T0.translation();
            q_cmd = Eigen::Quaterniond(T0.rotation());
            inited = true;
          }
          // 取最新 UDP 目标；无目标 / idle 超时 → 保持 p_cmd（原地 hold，不飞）
          Eigen::Vector3d p_tgt = p_cmd;
          Eigen::Quaterniond q_tgt = q_cmd;
          std::array<double, 16> tg;
          int64_t since = now_ms() - g_last_recv_ms.load();
          if (target.get(tg) && g_last_recv_ms.load() > 0 && since < 200) {
            Eigen::Affine3d Tt(Eigen::Matrix4d::Map(tg.data()));
            p_tgt = Tt.translation();
            q_tgt = Eigen::Quaterniond(Tt.rotation());
          }
          // 一阶 LPF 插值（平滑，C1 连续 → 杜绝 discontinuity reflex）
          Eigen::Vector3d dp = alpha * (p_tgt - p_cmd);
          double dn = dp.norm();
          if (dn > max_step) dp *= (max_step / dn);   // 每 tick 位移钳位
          p_cmd += dp;
          if (q_cmd.dot(q_tgt) < 0.0) q_tgt.coeffs() = -q_tgt.coeffs();
          q_cmd = q_cmd.slerp(alpha, q_tgt).normalized();

          Eigen::Affine3d T = Eigen::Affine3d::Identity();
          T.linear() = q_cmd.toRotationMatrix();
          T.translation() = p_cmd;
          std::array<double, 16> out{};
          Eigen::Matrix4d::Map(out.data()) = T.matrix();
          if (g_stop.load())
            return franka::MotionFinished(franka::CartesianPose(out));
          return franka::CartesianPose(out);
        });
        break;   // MotionFinished 正常退出
      } catch (const franka::Exception& e) {
        retries++;
        std::cerr << "❌ reflex (#" << retries << "/" << max_retries
                  << "): " << e.what() << "\n";
        try {
          robot.automaticErrorRecovery();
          std::cerr << "  recovered\n";
        } catch (const franka::Exception& e2) {
          std::cerr << "  automaticErrorRecovery 失败: " << e2.what() << "\n";
          if (std::string(e2.what()).find("safety function") != std::string::npos)
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
    g_stop.store(true);
    ee_writer.join();
    std::cerr << "✅ exited (retries=" << retries << ")\n";
  } catch (const franka::Exception& e) {
    std::cerr << "❌ " << e.what() << "\n";
    g_stop.store(true);
  }
  udp_thr.join();
  return 0;
}
