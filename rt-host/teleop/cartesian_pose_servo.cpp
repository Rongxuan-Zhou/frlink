// cartesian_pose_servo.cpp  (actually cartesian impedance torque control)
//
// Top-level logic:
//   • The controller returns franka::Torques (not CartesianPose)
//   • Cartesian impedance PD law implemented ourselves with Eigen: tau = J^T (−K Δp − D ẋ) + coriolis
//   • libfranka no longer does IK internally, eliminating the whole cartesian_motion_generator_* reflex family
//   • UDP receives an external 4×4 target, second-order cascaded filter to the desired pose
//   • reflexes only come from the absolute joint-torque limit or collisions (far more lenient than motion_generator)
//
// Differences from the original cartesian_impedance_control.cpp example:
//   • equilibrium pose is not fixed = init; it is updated dynamically from UDP → the user drags the arm along
//   • second-order filter added to avoid impedance jitter caused by the 90Hz UDP input
//   • reflex retry + freeze framework added (although reflexes should be far fewer under this architecture)
//
// Usage:
//   cartesian_pose_servo <robot-ip> [--port 50001]
//                                   [--alpha 0.005]   second-order filter coefficient
//                                   [--K-t 150]       translational stiffness N/m
//                                   [--K-r 10]        rotational stiffness Nm/rad

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
std::atomic<int64_t> g_last_recv_ms{0};   // used for user let-go detection
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
  double alpha = 0.050;   // 0.03 → 0.05: ~70% faster response (latency 50ms→30ms)
  double K_t = 1000.0;    // 500 → 1000: truly stiff, the contact-rich tracking the user wants
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

    // In torque control mode the collision thresholds differ slightly from cartesian pose mode
    // Set higher values to give the user margin to push the arm
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

    // Current EE sharing: lets 02 read the robot's actual position during calibration (instead of relying on the 02 accumulator)
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

    // Second-order cascaded filter state
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
    // Impedance parameters (free-space nominal values; variable impedance softens them on contact)
    // ══════════════════════════════════════════════════════════════════
    // K_free = K_t (cmd-line), K_contact = K_t * 0.25 (softened 4×)
    const double K_t_contact_ratio = 0.25;
    const double K_r_contact_ratio = 0.25;

    // F/T trigger parameters (external end-effector force estimated by O_F_ext_hat_K)
    // Variable impedance only kicks in on strong contact (interpolated between 10-25N), without disturbing stiff tracking
    const double F_low = 10.0;       // softening starts only when |F|>10N
    const double F_high = 25.0;      // fully softened when |F|>25N
    const double F_emergency = 35.0; // emergency freeze when |F|>35N
    // With K=1000 stiff PD, a 30cm EE lever arm × a few N → 5-7 Nm normally; the old 6Nm was too sensitive
    // Raised to 20Nm: still catches real anomalies (contact + long lever arm = real collision) without false alarms during normal operation
    const double T_emergency = 20.0;

    // Singularity-aware: FR3 home w ≈ 0.07-0.10, critical w ≈ 0.01. Thresholds moved down
    // so K stays at full scale throughout the home region and only shrinks when the elbow is extremely straight
    const double w_min = 0.005;      // almost singular
    const double w_full = 0.015;     // above this K is at full scale (home is far above this value)

    // FR3 joint limits (datasheet)
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
    // hard_cap is deprecated: null-space projection is used instead (does not break task coordination)

    // Time of the last emergency freeze, used to throttle printing
    int64_t last_emergency_log_ms = 0;
    // emergency hold period: for 1500ms after triggering, force freeze + drop UDP
    // Prevents 02 continuously pushing last_target from immediately overwriting the emergency hold
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
      // Expose the robot's actual EE to the 02 calibration mode (lock-free triple buffer)
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

      // User let-go detection: no UDP packet received for 200ms → make target_raw follow the current EE,
      // so the PD error → 0 and the arm stops immediately instead of "drifting" to the Python accumulator's last value
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

      // ── Second-order cascaded filter raw_target → raw_smooth → filtered ──
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

      // ── Cartesian impedance PD law (reused from libfranka's own cartesian_impedance_control example) ──
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
      // [Fix 1] F/T feedback in the loop: emergency stop + monitoring
      // ════════════════════════════════════════════════════════════════
      Eigen::Map<const Eigen::Matrix<double, 6, 1>> F_ext(state.O_F_ext_hat_K.data());
      double F_force_norm = F_ext.head(3).norm();
      double F_torque_norm = F_ext.tail(3).norm();
      bool emergency = (F_force_norm > F_emergency) || (F_torque_norm > T_emergency);
      if (emergency) {
        // Reuse the freeze mechanism so udp_listener drops subsequent UDP and no longer overwrites the emergency hold
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
        // Emergency period over + F/T recovered → release freeze so UDP takes effect again
        g_freeze.store(false);
        emergency_until_ms = 0;
        std::cerr << "🟢 emergency hold released, UDP resumed" << std::endl;
      }

      // ════════════════════════════════════════════════════════════════
      // [Fix 3] Singularity-aware: scale K down using manipulability w
      // w = sqrt(det(J J^T)); when the elbow is straight w → 0, J^T is ill-conditioned → K must be reduced
      // ════════════════════════════════════════════════════════════════
      Eigen::Matrix<double, 6, 6> JJT = jacobian * jacobian.transpose();
      double det_JJT = JJT.determinant();
      double w = (det_JJT > 0) ? std::sqrt(det_JJT) : 0.0;
      double k_sing_scale = std::clamp((w - w_min) / (w_full - w_min), 0.0, 1.0);

      // ════════════════════════════════════════════════════════════════
      // [Fix 2] Variable impedance: F/T detects contact intensity → K interpolated within [K_contact, K_free]
      // ════════════════════════════════════════════════════════════════
      double contact_alpha = std::clamp((F_force_norm - F_low) / (F_high - F_low), 0.0, 1.0);
      double K_t_now = ((1.0 - contact_alpha) + contact_alpha * K_t_contact_ratio) * K_t * k_sing_scale;
      double K_r_now = ((1.0 - contact_alpha) + contact_alpha * K_r_contact_ratio) * K_r * k_sing_scale;
      // Safety lower bound to avoid numerical singularity
      K_t_now = std::max(K_t_now, 10.0);
      K_r_now = std::max(K_r_now, 1.0);

      // Fixed-size Eigen to avoid heap-allocation jitter inside the 1kHz control loop
      Eigen::Matrix<double, 6, 6> stiff_now = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 6> damp_now  = Eigen::Matrix<double, 6, 6>::Zero();
      stiff_now.topLeftCorner<3, 3>() = K_t_now * Eigen::Matrix3d::Identity();
      stiff_now.bottomRightCorner<3, 3>() = K_r_now * Eigen::Matrix3d::Identity();
      // critical damping = 2*sqrt(K*M_eff). FR3 end effector incl. Hand has an effective mass ≈ 4-6 kg
      // Previously 2*sqrt(K) was used, equivalent to assuming M=1; at K=1000 that is only 45% of critical damping → chatter
      // M_eff taken as 5.0 so the damping matches the actual mass
      const double M_eff_t = 5.0;   // translational effective mass
      const double M_eff_r = 0.3;   // rotational effective moment of inertia (Franka EE inertia)
      damp_now.topLeftCorner<3, 3>() = 2.0 * std::sqrt(K_t_now * M_eff_t) * Eigen::Matrix3d::Identity();
      damp_now.bottomRightCorner<3, 3>() = 2.0 * std::sqrt(K_r_now * M_eff_r) * Eigen::Matrix3d::Identity();

      Eigen::Matrix<double, 7, 1> tau_task = jacobian.transpose() *
                                 (-stiff_now * error - damp_now * (jacobian * dq));

      // ════════════════════════════════════════════════════════════════
      // [Fix 4] Task-level constraint: jlim avoidance projected into the null space of J, does not break the task
      // N = I - J^+ J , J^+ uses the DLS damped pseudo-inverse to keep N well-conditioned near singularities
      // ════════════════════════════════════════════════════════════════
      // DLS damping coefficient: λ=0.05 with FR3 σ_min ≈ 0.05-0.1 gives an amplification of 8-10×, letting the N(J)
      // projection leak 5-10% into the range space of J, so the jlim push-back drags the end effector away. Khatib '95 + the libfranka
      // examples' standard choice is 0.10-0.20; contact-rich must lean toward the wide side.
      const double dls_lambda = 0.15;
      Eigen::Matrix<double, 6, 6> JJT_damped =
          JJT + (dls_lambda * dls_lambda) * Eigen::Matrix<double, 6, 6>::Identity();
      Eigen::Matrix<double, 7, 6> J_pinv = jacobian.transpose() * JJT_damped.inverse();
      Eigen::Matrix<double, 7, 7> N_proj =
          Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * jacobian;

      // Joint-limit potential (per-joint, projected into the null space below)
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
      // Project into the null space: keeps jlim from fighting the task
      Eigen::Matrix<double, 7, 1> tau_jlim_ns = N_proj * tau_jlim_raw;

      Eigen::Matrix<double, 7, 1> tau_d = tau_task + tau_jlim_ns + coriolis;

      // Per-joint absolute torque clamp (FR3 hardware limits J1-4: 87 Nm, J5-7: 12 Nm)
      // Prevents jlim_K=400 from requesting more than 12 Nm on the small joints J5/J6/J7 and triggering a joint_torque reflex
      static const std::array<double, 7> kTauMax = {{87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}};
      const double kTauSafeFraction = 0.85;  // keep a 15% safety margin
      for (int i = 0; i < 7; ++i) {
        double cap = kTauMax[i] * kTauSafeFraction;
        if (tau_d[i] > cap) tau_d[i] = cap;
        else if (tau_d[i] < -cap) tau_d[i] = -cap;
      }

      std::array<double, 7> tau_array;
      Eigen::VectorXd::Map(&tau_array[0], 7) = tau_d;

      // libfranka's own torque rate limiter: hard-limits dτ/dt between two adjacent frames
      // Eliminates the controller_torque_discontinuity family of reflexes
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
        // torque control is the default mode (no ControllerMode argument passed)
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
          std::cerr << "  max retries reached, giving up" << std::endl;
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
          std::cerr << "  automaticErrorRecovery failed: " << e2.what() << std::endl;
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
