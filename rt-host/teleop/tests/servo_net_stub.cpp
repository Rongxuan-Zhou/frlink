// tests/servo_net_stub.cpp - robot-free stand-in for the FR3 servos' network/RT plumbing.
//
// Links the SAME rt/*.hpp units as cartesian_pose_servo*.cpp: command socket + CmdFilter +
// listener thread, TripleBuffers, a fake 1 kHz loop (clock_nanosleep) applying the 200 ms hold
// to a dummy pose, the state writer threads (ee/wrench 20 Hz, joint 10 Hz, link 5 Hz, init
// pose once + re-sent every 1 s), the FRST1 publisher and the rt setup flags.
// SIGTERM/SIGINT -> clean exit 0 within ~150 ms. Exit 1 on bad arguments / bind failure.
//
//   servo_net_stub [--port P|--cmd-port P] [--cmd-bind IP] [--cmd-allow IPS] [--state-dst IP:PORT]
//                  [--state-dir DIR] [--state-files 0|1] [--rt-cpu N] [--aux-cpus LIST]
//                  [--check-rt] [--help]
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "../rt/cli.hpp"
#include "../rt/clock.hpp"
#include "../rt/rt_setup.hpp"
#include "../rt/state_pub.hpp"
#include "../rt/tick_stats.hpp"
#include "../rt/triple_buffer.hpp"
#include "../rt/udp_cmd.hpp"
#include "../rt/writers.hpp"

namespace {

std::atomic<bool> g_stop{false};
std::atomic<bool> g_freeze{false};
std::atomic<bool> g_recovering{false};
std::atomic<uint64_t> g_packet_count{0};
std::atomic<uint64_t> g_freeze_drops{0};
std::atomic<uint64_t> g_reflex_count{0};
std::atomic<int64_t> g_last_recv_ms{0};

using Pose16 = std::array<double, 16>;
using Wrench6 = std::array<double, 6>;
using Joint28 = std::array<double, 28>;

void on_sigint(int) { g_stop.store(true); }

void usage(const char* prog) {
  std::cout << "Usage: " << prog << " [--port P] [rt flags]\n" << rt::rt_flags_help();
}

}  // namespace

int main(int argc, char** argv) {
  rt::process_epoch_ns();  // freeze "process start" for the FRST1 epoch field
  rt::RtCliOptions cli;
  std::string err;
  if (!rt::prescan_rt_args(argc, argv, &cli, &err)) {
    std::cerr << "argument error: " << err << "\n";
    usage(argv[0]);
    return 1;
  }
  if (cli.help) {
    usage(argv[0]);
    return 0;
  }
  std::vector<int> aux_cpus;
  if (!cli.aux_cpus.empty() && !rt::parse_cpu_list(cli.aux_cpus, &aux_cpus, &err)) {
    std::cerr << "argument error: " << err << "\n";
    return 1;
  }
  if (cli.check_rt) return rt::run_check_rt(std::cout, aux_cpus, cli.rt_cpu);

  argc = cli.rest_argc();
  argv = cli.rest_argv_ptr();
  int port = 50001;
  for (int i = 1; i < argc; i += 2) {  // stub takes no robot ip, so the legacy loop starts at 1
    if (i + 1 >= argc) break;
    std::string flag = argv[i];
    if (flag == "--port") port = std::stoi(argv[i + 1]);
  }

  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  setvbuf(stdout, nullptr, _IOLBF, 0);
  setvbuf(stderr, nullptr, _IOLBF, 0);

  // ---- RT setup (before any helper thread exists) ----
  rt::RtSetupResult rts = rt::rt_setup_pre_threads(aux_cpus);
  if (!rts.mlock_ok) std::cerr << "warning: " << rts.mlock_err << "\n";
  if (rts.aux_requested && !rts.aux_ok) std::cerr << "warning: " << rts.aux_err << "\n";
  std::cout << "rt: mlockall=" << (rts.mlock_ok ? "ok" : "failed")
            << " prefault=" << rts.prefaulted_bytes
            << " aux=" << (rts.aux_requested ? rt::cpu_list_to_string(rts.aux_cpus) : "none") << std::endl;

  // ---- command socket + filter ----
  std::vector<uint32_t> allow;
  if (!rt::parse_allow_list(cli.cmd_allow, &allow, &err)) {
    std::cerr << "argument error: " << err << "\n";
    return 1;
  }
  rt::CommandSocket cmd_sock;
  if (!cmd_sock.open(cli.cmd_bind, port, &err)) {
    std::cerr << err << "\n";
    return 1;
  }
  rt::CmdFilter cmd_filter(allow);

  // ---- state outputs ----
  rt::StatePublisher pub;
  if (!cli.state_dst.empty() && !pub.open(cli.state_dst, &err)) {
    std::cerr << err << "\n";
    return 1;
  }
  rt::StateSink sink(cli.state_dir, cli.state_files, &pub);

  Pose16 init_pose{1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0.4, 0.0, 0.3, 1};
  const std::string init_body = rt::format_doubles(init_pose.data(), 16);
  sink.emit("franka_init_pose.txt", init_body);
  {
    Joint28 j{};
    const double q0[7] = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    for (int i = 0; i < 7; ++i) j[i] = q0[i];
    sink.emit("franka_joint_state.txt", rt::format_doubles(j.data(), 28));
  }

  rt::TripleBuffer<Pose16> target_raw(init_pose);
  rt::TripleBuffer<Pose16> current_ee(init_pose);
  rt::TripleBuffer<Wrench6> current_wrench;
  rt::TripleBuffer<Joint28> current_joint;
  rt::TickStats tick_stats;

  rt::CmdListenerHooks hooks{&g_stop, &g_freeze, &g_freeze_drops, &g_packet_count, &g_last_recv_ms};
  std::thread udp_thr([&] { rt::set_thread_name("udp_cmd"); rt::run_cmd_listener(cmd_sock, cmd_filter, target_raw, hooks); });
  std::thread ee_thr([&] { rt::set_thread_name("ee_writer"); rt::run_periodic_emit<16>(g_stop, 50, "franka_current_ee.txt", current_ee, sink); });
  std::thread wrench_thr([&] { rt::set_thread_name("wrench_writer"); rt::run_periodic_emit<6>(g_stop, 50, "franka_wrench.txt", current_wrench, sink); });
  std::thread joint_thr([&] { rt::set_thread_name("joint_writer"); rt::run_periodic_emit<28>(g_stop, 100, "franka_joint_state.txt", current_joint, sink); });
  std::thread init_thr([&] { rt::set_thread_name("init_resend"); rt::run_init_pose_resender(g_stop, 1000, init_body, sink); });
  rt::LinkSources link_src{&cmd_filter, &tick_stats, &g_freeze, &g_recovering, &g_reflex_count};
  std::thread link_thr([&] { rt::set_thread_name("link_writer"); rt::run_link_writer(g_stop, link_src, sink, 200); });

  // ---- pin the "control" thread (this one) after the helpers exist ----
  if (cli.rt_cpu >= 0) {
    if (!rt::pin_self(cli.rt_cpu, &err)) std::cerr << "warning: " << err << "\n";
    else std::cout << "rt: control thread pinned to cpu " << cli.rt_cpu << std::endl;
  }
  {  // mimic libfranka's ControlLoop: the control thread runs SCHED_FIFO (warn if not granted)
    sched_param sp{};
    sp.sched_priority = 80;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != 0) std::cerr << "warning: SCHED_FIFO 80 not granted (" << strerror(rc) << ")\n";
    else std::cout << "rt: control thread SCHED_FIFO 80" << std::endl;
  }

  std::cout << "-> command socket " << cli.cmd_bind << ":" << port << " allow=" << cli.cmd_allow
            << "  state dir=" << cli.state_dir << " files=" << (cli.state_files ? 1 : 0)
            << " dst=" << (cli.state_dst.empty() ? "none" : cli.state_dst) << std::endl;
  std::cout << "-> fake 1 kHz loop running; SIGTERM to stop" << std::endl;

  // ---- fake 1 kHz loop with the servo's target-selection logic ----
  Pose16 filtered = init_pose;         // pretend robot pose (O_T_EE)
  Pose16 rt_override = init_pose;
  bool rt_override_active = false;
  uint64_t rt_seen_pkts = 0;
  uint64_t tick = 0;
  timespec next{};
  clock_gettime(CLOCK_MONOTONIC, &next);
  while (!g_stop.load()) {
    next.tv_nsec += 1000000;
    if (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec += 1; }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

    tick_stats.on_tick(rt::mono_now_ns());
    tick++;
    current_ee.write(filtered);
    { Wrench6 w{}; current_wrench.write(w); }
    { Joint28 j{}; j[0] = 0.001 * static_cast<double>(tick % 1000); current_joint.write(j); }

    const int64_t now_ms = rt::steady_now_ms();
    const int64_t since_recv = now_ms - g_last_recv_ms.load();
    {
      const uint64_t pc = g_packet_count.load(std::memory_order_acquire);
      if (pc != rt_seen_pkts) { rt_seen_pkts = pc; rt_override_active = false; }
    }
    if (since_recv > 200 && g_last_recv_ms.load() > 0) {   // 200 ms hold: target := current pose
      rt_override = filtered;
      rt_override_active = true;
    }
    const Pose16 raw_target = rt_override_active ? rt_override : target_raw.read();
    for (int i = 12; i < 15; ++i) filtered[i] += 0.05 * (raw_target[i] - filtered[i]);
  }

  g_stop.store(true);
  udp_thr.join();
  ee_thr.join();
  wrench_thr.join();
  joint_thr.join();
  init_thr.join();
  link_thr.join();
  std::cout << "exited (tick=" << tick << " udp=" << g_packet_count.load()
            << " accepted=" << cmd_filter.accepted.load() << ")" << std::endl;
  return 0;
}
