// cartesian_pose_servo.cpp  (实际是 cartesian impedance torque control)
//
// 顶层逻辑：
//   • 控制器返回 franka::Torques（不是 CartesianPose）
//   • 自己用 Eigen 实现笛卡尔阻抗 PD 律：tau = J^T (−K Δp − D ẋ) + coriolis
//   • libfranka 内部不再做 IK，杜绝 cartesian_motion_generator_* 整族 reflex
//   • UDP 接外部 4×4 target，二阶级联滤波到 desired pose
//   • reflex 仅来自关节力矩绝对上限或碰撞（远比 motion_generator 宽松）
//
// 跟原 cartesian_impedance_control.cpp 例子的差异：
//   • equilibrium pose 不固定 = init，而是从 UDP 动态更新 → 用户拖着机械臂走
//   • 加二阶滤波避免 90Hz UDP 输入造成阻抗抖
//   • 加 reflex retry + freeze 框架（虽然这套架构下 reflex 应大幅减少）
//
// 用法：
//   cartesian_pose_servo <robot-ip> [--port 50001]
//                                   [--alpha 0.005]   二阶滤波系数
//                                   [--K-t 150]       平移刚度 N/m
//                                   [--K-r 10]        旋转刚度 Nm/rad

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <Eigen/Dense>

#include <franka/control_tools.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "examples_common.h"

// PART B: RT/network units (header-only, teleop/rt/)
#include "rt/cli.hpp"
#include "rt/clock.hpp"
#include "rt/rt_setup.hpp"
#include "rt/state_pub.hpp"
#include "rt/tick_stats.hpp"
#include "rt/triple_buffer.hpp"
#include "rt/udp_cmd.hpp"
#include "rt/writers.hpp"

namespace {

constexpr int kPoseFloats = 16;
constexpr int kPoseBytes = kPoseFloats * sizeof(double);

std::atomic<bool> g_stop{false};
std::atomic<bool> g_freeze{false};
std::atomic<uint64_t> g_packet_count{0};
std::atomic<uint64_t> g_freeze_drops{0};
std::atomic<int64_t> g_last_recv_ms{0};   // user 松手检测用
std::atomic<bool> g_recovering{false};      // PART B: true from a reflex until robot.control() re-enters
std::atomic<uint64_t> g_reflex_count{0};    // PART B: mirrors total_reflex for the link writer

// PART B: the mutex-guarded AtomicPose/AtomicJoint/AtomicWrench structs are replaced by
// lock-free single-writer/single-reader rt::TripleBuffer<T> instances (no locks in the tick).
using Pose16 = std::array<double, 16>;
using Joint28 = std::array<double, 28>;
using Wrench6 = std::array<double, 6>;

void on_sigint(int) { g_stop.store(true); }

// PART B: udp_listener() moved to rt::run_cmd_listener (rt/udp_cmd.hpp): recvfrom + allow-list
// + sender latch; identical freeze / g_packet_count / g_last_recv_ms semantics.

}  // namespace

int main(int argc, char** argv) {
  rt::process_epoch_ns();  // PART B: FRST1 epoch = process start
  rt::RtCliOptions rtcli;
  std::string rt_err;
  if (!rt::prescan_rt_args(argc, argv, &rtcli, &rt_err)) {
    std::cerr << "argument error: " << rt_err << "\n";
    return 1;
  }
  if (rtcli.help) {
    std::cout << "Usage: " << argv[0]
              << " <robot-ip> [--port P] [--alpha A] [--K-t Kt] [--K-r Kr]\n"
              << rt::rt_flags_help();
    return 0;
  }
  std::vector<int> aux_cpus;
  if (!rtcli.aux_cpus.empty() && !rt::parse_cpu_list(rtcli.aux_cpus, &aux_cpus, &rt_err)) {
    std::cerr << "argument error: " << rt_err << "\n";
    return 1;
  }
  if (rtcli.check_rt) return rt::run_check_rt(std::cout, aux_cpus, rtcli.rt_cpu);
  argc = rtcli.rest_argc();       // the legacy loop below only ever sees the legacy flags
  argv = rtcli.rest_argv_ptr();
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <robot-ip> [--port P] [--alpha A] [--K-t Kt] [--K-r Kr]\n";
    return 1;
  }
  std::string robot_ip = argv[1];
  int port = 50001;
  double alpha = 0.050;   // 0.03 → 0.05：响应快 ~70%（latency 50ms→30ms）
  double K_t = 1000.0;    // 500 → 1000：真 stiff，user 想要的 contact-rich 跟随
  double K_r = 80.0;      // 40 → 80
  for (int i = 2; i < argc; i += 2) {
    if (i + 1 >= argc) break;
    std::string flag = argv[i];
    if (flag == "--port") port = std::stoi(argv[i + 1]);
    else if (flag == "--alpha") alpha = std::stod(argv[i + 1]);
    else if (flag == "--K-t") K_t = std::stod(argv[i + 1]);
    else if (flag == "--K-r") K_r = std::stod(argv[i + 1]);
  }

  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  setvbuf(stdout, nullptr, _IOLBF, 0);
  setvbuf(stderr, nullptr, _IOLBF, 0);

  // PART B: RT setup before any helper thread exists (mlockall, 8 MB stack prefault, aux mask).
  rt::RtSetupResult rts = rt::rt_setup_pre_threads(aux_cpus);
  if (!rts.mlock_ok) std::cerr << "warning: " << rts.mlock_err << std::endl;
  if (rts.aux_requested && !rts.aux_ok) std::cerr << "warning: " << rts.aux_err << std::endl;
  std::cout << "  rt: mlockall=" << (rts.mlock_ok ? "ok" : "failed")
            << " prefault=" << rts.prefaulted_bytes
            << " aux=" << (rts.aux_requested ? rt::cpu_list_to_string(rts.aux_cpus) : "none")
            << " rt_cpu=" << rtcli.rt_cpu << std::endl;
  // PART B: command socket + filter + state outputs. A bind failure is now fatal (exit 1);
  // the legacy listener thread silently returned and the servo ran without commands.
  std::vector<uint32_t> cmd_allow;
  if (!rt::parse_allow_list(rtcli.cmd_allow, &cmd_allow, &rt_err)) {
    std::cerr << "argument error: " << rt_err << std::endl;
    return 1;
  }
  rt::CommandSocket cmd_sock;
  if (!cmd_sock.open(rtcli.cmd_bind, port, &rt_err)) {
    std::cerr << rt_err << std::endl;
    return 1;
  }
  rt::CmdFilter cmd_filter(cmd_allow);
  rt::StatePublisher state_pub;
  if (!rtcli.state_dst.empty() && !state_pub.open(rtcli.state_dst, &rt_err)) {
    std::cerr << rt_err << std::endl;
    return 1;
  }
  rt::StateSink sink(rtcli.state_dir, rtcli.state_files, &state_pub);
  std::cout << "  state: dir=" << rtcli.state_dir << " files=" << (rtcli.state_files ? 1 : 0)
            << " dst=" << (rtcli.state_dst.empty() ? "none" : rtcli.state_dst) << std::endl;

  std::cout << "→ Connecting to " << robot_ip << " ..." << std::endl;
  std::cout << "  K_t=" << K_t << " N/m, K_r=" << K_r << " Nm/rad, alpha=" << alpha
            << std::endl;

  try {
    franka::Robot robot(robot_ip, franka::RealtimeConfig::kIgnore);
    setDefaultBehavior(robot);
    try {
      robot.automaticErrorRecovery();
      std::cout << "  automaticErrorRecovery OK" << std::endl;
    } catch (const franka::Exception& e) {
      std::cerr << "  automaticErrorRecovery skipped: " << e.what() << std::endl;
    }

    constexpr double kMLoad = 0.25;
    std::array<double, 3> kCom{0.0, 0.0, 0.05};
    std::array<double, 9> kInertia{1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4};
    robot.setLoad(kMLoad, kCom, kInertia);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // torque control 模式下 collision threshold 跟笛卡尔位姿模式略不同
    // 设较高值给 user 推机械臂的余量
    robot.setCollisionBehavior({{30, 30, 30, 30, 30, 30, 30}}, {{30, 30, 30, 30, 30, 30, 30}},
                               {{30, 30, 30, 30, 30, 30}}, {{30, 30, 30, 30, 30, 30}});

    franka::Model model = robot.loadModel();
    franka::RobotState init = robot.readOnce();
    std::array<double, 16> init_pose = init.O_T_EE;
    std::cout << "  init O_T_EE pos = ("
              << init_pose[12] << "," << init_pose[13] << "," << init_pose[14] << ")"
              << std::endl;
    // PART B: same bytes as before, but through the shared formatter + sink (file + datagram)
    const std::string init_pose_body = rt::format_doubles(init_pose.data(), 16);
    sink.emit("franka_init_pose.txt", init_pose_body);
    std::cout << "  init pose saved to " << rtcli.state_dir << "/franka_init_pose.txt" << std::endl;

    // PART B: target_raw is written ONLY by the udp thread and read ONLY by the control tick.
    rt::TripleBuffer<Pose16> target_raw(init_pose);
    rt::CmdListenerHooks cmd_hooks{&g_stop, &g_freeze, &g_freeze_drops, &g_packet_count,
                                   &g_last_recv_ms};
    std::thread udp_thr([&]() {
      rt::set_thread_name("udp_cmd");
      rt::run_cmd_listener(cmd_sock, cmd_filter, target_raw, cmd_hooks);
    });

    // 当前 EE 共享：让 02 标定时能读到 robot 实际位置（不靠 02 累加器）
    // PART B: combined 20 Hz writer kept (EE, wrench, joint back-to-back); each body goes
    // through the shared formatter + sink (file + FRST1 datagram).
    rt::TripleBuffer<Pose16> current_ee(init_pose);
    rt::TripleBuffer<Wrench6> current_wrench;   // W1 wrench expose (written in control_cb)
    rt::TripleBuffer<Joint28> current_joint;    // W1 round-2 joint state expose
    std::thread ee_writer_thr([&]() {
      rt::set_thread_name("ee_writer");
      while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20Hz
        Pose16 ee = current_ee.read();
        sink.emit("franka_current_ee.txt", rt::format_doubles(ee.data(), 16));
        Wrench6 w = current_wrench.read();
        sink.emit("franka_wrench.txt", rt::format_doubles(w.data(), 6));
        Joint28 j = current_joint.read();
        sink.emit("franka_joint_state.txt", rt::format_doubles(j.data(), 28));
      }
    });
    // PART B: init pose re-sent as a datagram every 1 s; link stats file/datagram at 5 Hz.
    std::thread init_resend_thr([&]() {
      rt::set_thread_name("init_resend");
      rt::run_init_pose_resender(g_stop, 1000, init_pose_body, sink);
    });
    rt::TickStats tick_stats;
    rt::LinkSources link_src{&cmd_filter, &tick_stats, &g_freeze, &g_recovering, &g_reflex_count};
    std::thread link_writer_thr([&]() {
      rt::set_thread_name("link_writer");
      rt::run_link_writer(g_stop, link_src, sink, 200);
    });

    // 二阶级联滤波状态
    std::array<double, 16> raw_smooth = init_pose;
    std::array<double, 16> filtered = init_pose;
    // PART B: RT-side target override (200 ms hold / F-T emergency / reflex recovery). The tick
    // never writes target_raw (single-writer rule); it overrides locally and the override is
    // cancelled by the next packet that reaches target_raw — exactly the old "UDP overwrites
    // target_raw" ordering.
    std::array<double, 16> rt_override = init_pose;
    bool rt_override_active = false;
    uint64_t rt_seen_pkts = 0;
    const double alpha_raw = alpha * 2.0;
    const double alpha_filt = alpha;

    // ══════════════════════════════════════════════════════════════════
    // 阻抗参数（free-space 标称值；contact 时变阻抗会软化）
    // ══════════════════════════════════════════════════════════════════
    // K_free = K_t (cmd-line)，K_contact = K_t * 0.25 (软化 4×)
    const double K_t_contact_ratio = 0.25;
    const double K_r_contact_ratio = 0.25;

    // F/T 触发参数（O_F_ext_hat_K 估计的末端外力）
    // 变阻抗仅在大力接触时启动 (10-25N 间插值)，不打扰 stiff 跟随
    const double F_low = 10.0;       // |F|>10N 才开始软化
    const double F_high = 25.0;      // |F|>25N 完全软化
    const double F_emergency = 35.0; // |F|>35N 紧急冻结
    // K=1000 stiff PD 让 EE 杠杆臂 30cm × 几 N → 正常 5-7 Nm，旧 6Nm 太敏感
    // 抬到 20Nm：仍能捕获真正异常（接触 + 长杠杆 = 真碰撞），不误报正常操作
    const double T_emergency = 20.0;

    // Singularity-aware：FR3 home w ≈ 0.07-0.10，临界 w ≈ 0.01。把阈值下移
    // 让 home 区域 K 永远满刻度，仅在极端 elbow 伸直时缩 K
    const double w_min = 0.005;      // 几乎奇异
    const double w_full = 0.015;     // 这以上 K 满刻度（home 远超此值）

    // FR3 关节限位 (datasheet)
    struct JL { double lo, hi; };
    const std::array<JL, 7> jlim = {{
        {-2.7437, +2.7437},  // J1
        {-1.7837, +1.7837},  // J2
        {-2.9007, +2.9007},  // J3
        {-3.0421, -0.1518},  // J4
        {-2.8065, +2.8065},  // J5
        {+0.5445, +4.5169},  // J6
        {-3.0159, +3.0159},  // J7
    }};
    const double jlim_margin = 0.50;
    const double jlim_K = 400.0;
    const double jlim_D = 25.0;
    // hard_cap 已弃用：现在用零空间投影代替（不破坏 task coordination）

    // 上次紧急冻结时间，用于打印 throttle
    int64_t last_emergency_log_ms = 0;
    // emergency 持续期：触发后 1500ms 内强制 freeze + UDP 丢弃
    // 避免 02 持续推 last_target 把 emergency hold 立即覆盖
    int64_t emergency_until_ms = 0;
    const int64_t kEmergencyHoldMs = 1500;

    std::cout << "→ UDP listener on " << rtcli.cmd_bind << ":" << port
              << " allow=" << rtcli.cmd_allow << std::endl;
    std::cout << "→ Entering 1kHz cartesian-impedance torque control. Ctrl-C to stop."
              << std::endl;

    uint64_t tick = 0;

    auto control_cb = [&](const franka::RobotState& state,
                          franka::Duration /*period*/) -> franka::Torques {
      tick_stats.on_tick(rt::mono_now_ns());  // PART B: entry-interval stats (no alloc/IO/locks)
      tick++;
      // 把 robot 实际 EE 暴露给 02 标定模式（lock-free triple buffer）
      current_ee.write(state.O_T_EE);
      // W1 wrench expose: also set wrench for franka_wrench.txt (20Hz writer reads it)
      current_wrench.write(state.O_F_ext_hat_K);
      // W1 round-2 joint state: q + dq + tau_J + tau_ext_hat_filtered → franka_joint_state.txt
      {
        Joint28 jbuf;
        for (int i = 0; i < 7; ++i) {
          jbuf[i]      = state.q[i];
          jbuf[i + 7]  = state.dq[i];
          jbuf[i + 14] = state.tau_J[i];
          jbuf[i + 21] = state.tau_ext_hat_filtered[i];
        }
        current_joint.write(jbuf);
      }

      // user 松手检测：UDP 200ms 没收到包 → 让 target_raw 跟随当前 EE，
      // 这样 PD error → 0，机械臂立刻停下不再"飘"到 Python 累加器最后值
      auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      int64_t since_recv = now_ms - g_last_recv_ms.load();
      {
        // PART B: a packet that reached target_raw since the last tick cancels the override
        const uint64_t pc = g_packet_count.load(std::memory_order_acquire);
        if (pc != rt_seen_pkts) {
          rt_seen_pkts = pc;
          rt_override_active = false;
        }
      }
      if (since_recv > 200 && g_last_recv_ms.load() > 0) {
        rt_override = state.O_T_EE;        // was: target_raw.set(state.O_T_EE)
        rt_override_active = true;
      }

      std::array<double, 16> raw_target = rt_override_active ? rt_override : target_raw.read();

      // ── 二阶级联滤波 raw_target → raw_smooth → filtered ──
      Eigen::Map<const Eigen::Matrix<double, 4, 4>> rt_mat(raw_target.data());
      Eigen::Map<Eigen::Matrix<double, 4, 4>> rs_mat(raw_smooth.data());
      Eigen::Quaterniond q_rt(rt_mat.topLeftCorner<3, 3>());
      Eigen::Quaterniond q_rs(rs_mat.topLeftCorner<3, 3>());
      if (q_rt.dot(q_rs) < 0) q_rt.coeffs() *= -1.0;

      raw_smooth[12] += alpha_raw * (raw_target[12] - raw_smooth[12]);
      raw_smooth[13] += alpha_raw * (raw_target[13] - raw_smooth[13]);
      raw_smooth[14] += alpha_raw * (raw_target[14] - raw_smooth[14]);
      Eigen::Quaterniond q_rs_new = q_rs.slerp(alpha_raw, q_rt);
      Eigen::Matrix3d R_rs = q_rs_new.toRotationMatrix();
      for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r) raw_smooth[c * 4 + r] = R_rs(r, c);
      raw_smooth[3] = raw_smooth[7] = raw_smooth[11] = 0.0;
      raw_smooth[15] = 1.0;

      Eigen::Map<Eigen::Matrix<double, 4, 4>> filt_mat(filtered.data());
      Eigen::Quaterniond q_filt(filt_mat.topLeftCorner<3, 3>());
      Eigen::Quaterniond q_rs2(rs_mat.topLeftCorner<3, 3>());
      if (q_rs2.dot(q_filt) < 0) q_rs2.coeffs() *= -1.0;
      filtered[12] += alpha_filt * (raw_smooth[12] - filtered[12]);
      filtered[13] += alpha_filt * (raw_smooth[13] - filtered[13]);
      filtered[14] += alpha_filt * (raw_smooth[14] - filtered[14]);
      Eigen::Quaterniond q_filt_new = q_filt.slerp(alpha_filt, q_rs2);
      Eigen::Matrix3d R_filt = q_filt_new.toRotationMatrix();
      for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r) filtered[c * 4 + r] = R_filt(r, c);
      filtered[3] = filtered[7] = filtered[11] = 0.0;
      filtered[15] = 1.0;

      // ── 笛卡尔阻抗 PD 律 (复用 libfranka 自带 cartesian_impedance_control 例子) ──
      Eigen::Affine3d desired_T(Eigen::Matrix4d::Map(filtered.data()));
      Eigen::Vector3d position_d(desired_T.translation());
      Eigen::Quaterniond orientation_d(desired_T.rotation());

      Eigen::Affine3d cur_T(Eigen::Matrix4d::Map(state.O_T_EE.data()));
      Eigen::Vector3d position(cur_T.translation());
      Eigen::Quaterniond orientation(cur_T.rotation());

      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d;
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      Eigen::Quaterniond error_quat(orientation.inverse() * orientation_d);
      error.tail(3) << error_quat.x(), error_quat.y(), error_quat.z();
      error.tail(3) << -cur_T.rotation() * error.tail(3);

      auto coriolis_array = model.coriolis(state);
      auto jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());

      // ════════════════════════════════════════════════════════════════
      // [Fix 1] F/T 反馈进环：紧急停 + 监控
      // ════════════════════════════════════════════════════════════════
      Eigen::Map<const Eigen::Matrix<double, 6, 1>> F_ext(state.O_F_ext_hat_K.data());
      double F_force_norm = F_ext.head(3).norm();
      double F_torque_norm = F_ext.tail(3).norm();
      bool emergency = (F_force_norm > F_emergency) || (F_torque_norm > T_emergency);
      if (emergency) {
        // 复用 freeze 机制让 udp_listener 丢弃后续 UDP，不再覆盖紧急 hold
        g_freeze.store(true);
        emergency_until_ms = now_ms + kEmergencyHoldMs;
        rt_override = state.O_T_EE;        // was: target_raw.set(state.O_T_EE)
        rt_override_active = true;
        if (now_ms - last_emergency_log_ms > 1000) {
          std::cerr << "🚨 F/T emergency: |F|=" << F_force_norm
                    << "N |T|=" << F_torque_norm << "Nm → freeze " << kEmergencyHoldMs << "ms"
                    << std::endl;
          last_emergency_log_ms = now_ms;
        }
      } else if (emergency_until_ms > 0 && now_ms > emergency_until_ms) {
        // 紧急期过 + F/T 已恢复 → 解 freeze 让 UDP 重新生效
        g_freeze.store(false);
        emergency_until_ms = 0;
        std::cerr << "🟢 emergency hold released, UDP resumed" << std::endl;
      }

      // ════════════════════════════════════════════════════════════════
      // [Fix 3] Singularity-aware：用 manipulability w 缩 K
      // w = sqrt(det(J J^T))，elbow 伸直时 w → 0，J^T 病态 → K 必须降
      // ════════════════════════════════════════════════════════════════
      Eigen::Matrix<double, 6, 6> JJT = jacobian * jacobian.transpose();
      double det_JJT = JJT.determinant();
      double w = (det_JJT > 0) ? std::sqrt(det_JJT) : 0.0;
      double k_sing_scale = std::clamp((w - w_min) / (w_full - w_min), 0.0, 1.0);

      // ════════════════════════════════════════════════════════════════
      // [Fix 2] 变阻抗：F/T 检测接触强度 → K 在 [K_contact, K_free] 间插值
      // ════════════════════════════════════════════════════════════════
      double contact_alpha = std::clamp((F_force_norm - F_low) / (F_high - F_low), 0.0, 1.0);
      double K_t_now = ((1.0 - contact_alpha) + contact_alpha * K_t_contact_ratio) * K_t * k_sing_scale;
      double K_r_now = ((1.0 - contact_alpha) + contact_alpha * K_r_contact_ratio) * K_r * k_sing_scale;
      // 安全下限避免数值奇异
      K_t_now = std::max(K_t_now, 10.0);
      K_r_now = std::max(K_r_now, 1.0);

      // 固定尺寸 Eigen 避免 1kHz 控制循环内堆分配 jitter
      Eigen::Matrix<double, 6, 6> stiff_now = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 6> damp_now  = Eigen::Matrix<double, 6, 6>::Zero();
      stiff_now.topLeftCorner<3, 3>() = K_t_now * Eigen::Matrix3d::Identity();
      stiff_now.bottomRightCorner<3, 3>() = K_r_now * Eigen::Matrix3d::Identity();
      // critical damping = 2*sqrt(K*M_eff)。FR3 末端含 Hand 等效质量 ≈ 4-6 kg
      // 之前用 2*sqrt(K) 等效假设 M=1，K=1000 时只够 45% critical damping → 抖振
      // M_eff 取 5.0 让阻尼匹配实际质量
      const double M_eff_t = 5.0;   // 平移等效质量
      const double M_eff_r = 0.3;   // 旋转等效转动惯量 (Franka EE inertia)
      damp_now.topLeftCorner<3, 3>() = 2.0 * std::sqrt(K_t_now * M_eff_t) * Eigen::Matrix3d::Identity();
      damp_now.bottomRightCorner<3, 3>() = 2.0 * std::sqrt(K_r_now * M_eff_r) * Eigen::Matrix3d::Identity();

      Eigen::Matrix<double, 7, 1> tau_task = jacobian.transpose() *
                                 (-stiff_now * error - damp_now * (jacobian * dq));

      // ════════════════════════════════════════════════════════════════
      // [Fix 4] 任务级约束：jlim avoidance 投影到 J 的零空间，不破坏 task
      // N = I - J^+ J ， J^+ 用 DLS 阻尼伪逆防 singular 时 N 病态
      // ════════════════════════════════════════════════════════════════
      // DLS 阻尼系数：λ=0.05 在 FR3 σ_min ≈ 0.05-0.1 时放大率达 8-10×，让 N(J)
      // 投影漏到 J 的 range 空间 5-10%，jlim 反推会拉走末端。Khatib '95 + libfranka
      // 范例的标准取法是 0.10-0.20，contact-rich 必须取偏宽。
      const double dls_lambda = 0.15;
      Eigen::Matrix<double, 6, 6> JJT_damped =
          JJT + (dls_lambda * dls_lambda) * Eigen::Matrix<double, 6, 6>::Identity();
      Eigen::Matrix<double, 7, 6> J_pinv = jacobian.transpose() * JJT_damped.inverse();
      Eigen::Matrix<double, 7, 7> N_proj =
          Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * jacobian;

      // 关节限位势能 (per-joint，待会儿投影到零空间)
      Eigen::Matrix<double, 7, 1> tau_jlim_raw;
      tau_jlim_raw.setZero();
      for (int i = 0; i < 7; ++i) {
        double qi = state.q[i];
        double qd_i = state.dq[i];
        double dist_lo = qi - jlim[i].lo;
        double dist_hi = jlim[i].hi - qi;
        if (dist_lo < jlim_margin) {
          double e = jlim_margin - dist_lo;
          tau_jlim_raw[i] += jlim_K * e * e;
          tau_jlim_raw[i] -= jlim_D * qd_i;
        }
        if (dist_hi < jlim_margin) {
          double e = jlim_margin - dist_hi;
          tau_jlim_raw[i] -= jlim_K * e * e;
          tau_jlim_raw[i] -= jlim_D * qd_i;
        }
      }
      // 投影到零空间：让 jlim 不打架 task
      Eigen::Matrix<double, 7, 1> tau_jlim_ns = N_proj * tau_jlim_raw;

      Eigen::Matrix<double, 7, 1> tau_d = tau_task + tau_jlim_ns + coriolis;

      // 每关节绝对 torque clamp（FR3 hardware 限位 J1-4: 87 Nm, J5-7: 12 Nm）
      // 防 jlim_K=400 在小关节 J5/J6/J7 请求超 12 Nm 触发 joint_torque reflex
      static const std::array<double, 7> kTauMax = {{87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}};
      const double kTauSafeFraction = 0.85;  // 留 15% 安全裕度
      for (int i = 0; i < 7; ++i) {
        double cap = kTauMax[i] * kTauSafeFraction;
        if (tau_d[i] > cap) tau_d[i] = cap;
        else if (tau_d[i] < -cap) tau_d[i] = -cap;
      }

      std::array<double, 7> tau_array;
      Eigen::VectorXd::Map(&tau_array[0], 7) = tau_d;

      // libfranka 自带 torque rate limiter：硬限相邻两帧 dτ/dt
      // 杜绝 controller_torque_discontinuity 这一族 reflex
      tau_array = franka::limitRate(franka::kMaxTorqueRate, tau_array, state.tau_J_d);

      franka::Torques out(tau_array);
      if (g_stop.load()) return franka::MotionFinished(out);
      return out;
    };

    // PART B: pin the control thread (this one — libfranka runs the callback in the caller and
    // makes it SCHED_FIFO itself) only AFTER every helper thread was spawned with the aux mask.
    if (rtcli.rt_cpu >= 0) {
      if (!rt::pin_self(rtcli.rt_cpu, &rt_err)) std::cerr << "warning: " << rt_err << std::endl;
      else std::cout << "  rt: control thread pinned to cpu " << rtcli.rt_cpu << std::endl;
    }

    const int max_retries = 50;
    int retries = 0;
    uint64_t total_reflex = 0;
    while (!g_stop.load()) {
      try {
        // torque control 默认 mode（不传 ControllerMode 参数）
        tick_stats.reset_interval();     // PART B: recovery gap is not a missed cycle
        g_recovering.store(false);
        robot.control(control_cb);
        break;
      } catch (const franka::Exception& e) {
        total_reflex++;
        g_reflex_count.store(total_reflex);   // PART B: link writer
        g_recovering.store(true);
        std::cerr << "❌ reflex (#" << total_reflex << " retry "
                  << retries + 1 << "/" << max_retries << "): " << e.what() << std::endl;
        if (g_stop.load()) break;
        if (++retries > max_retries) {
          std::cerr << "  max retries 已达，放弃" << std::endl;
          break;
        }
        try {
          g_freeze.store(true);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          robot.automaticErrorRecovery();
          franka::RobotState s = robot.readOnce();
          filtered = s.O_T_EE;
          raw_smooth = s.O_T_EE;
          rt_override = s.O_T_EE;          // was: target_raw.set(s.O_T_EE) (control loop idle here)
          rt_override_active = true;
          std::cerr << "  recovered, EE=("
                    << s.O_T_EE[12] << "," << s.O_T_EE[13] << "," << s.O_T_EE[14]
                    << "), freeze 2s" << std::endl;
          std::thread([&]() {
            if (!aux_cpus.empty()) rt::set_self_affinity(aux_cpus, nullptr);  // PART B: off rt cpu
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            g_freeze.store(false);
            std::cerr << "  unfreeze" << std::endl;
          }).detach();
        } catch (const franka::Exception& e2) {
          std::cerr << "  automaticErrorRecovery 失败: " << e2.what() << std::endl;
          break;
        }
      }
    }

    g_stop.store(true);
    udp_thr.join();
    if (ee_writer_thr.joinable()) ee_writer_thr.join();
    if (init_resend_thr.joinable()) init_resend_thr.join();
    if (link_writer_thr.joinable()) link_writer_thr.join();
    std::cout << "✅ exited (tick=" << tick
              << " udp=" << g_packet_count.load()
              << " total_reflex=" << total_reflex << ")" << std::endl;
    return 0;
  } catch (const franka::Exception& e) {
    std::cerr << "❌ franka::Exception: " << e.what() << std::endl;
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "❌ std::exception: " << e.what() << std::endl;
    return 3;
  }
}
