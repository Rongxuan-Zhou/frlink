// rt/writers.hpp - helper-thread loops shared by both servos and the network stub:
// periodic state emitters, the 1 s init-pose re-sender and the 5 Hz link writer.
#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "clock.hpp"
#include "state_pub.hpp"
#include "tick_stats.hpp"
#include "triple_buffer.hpp"
#include "udp_cmd.hpp"

namespace rt {

// sleep period_ms, read the latest value, emit "<v0> <v1> ... <vN-1>\n" (file + datagram).
template <size_t N>
inline void run_periodic_emit(const std::atomic<bool>& stop, int period_ms, const char* name,
                              TripleBuffer<std::array<double, N>>& src, StateSink& sink) {
  while (!stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    std::array<double, N> v = src.read();
    sink.emit(name, format_doubles(v.data(), static_cast<int>(N)));
  }
}

// Datagram-only re-send of the init pose body every period_ms (file was written once at start).
inline void run_init_pose_resender(const std::atomic<bool>& stop, int period_ms,
                                   const std::string& body, StateSink& sink) {
  int64_t next = steady_now_ms() + period_ms;
  while (!stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (steady_now_ms() >= next) {
      sink.publish_only("franka_init_pose.txt", body);
      next += period_ms;
    }
  }
}

struct LinkSources {
  const CmdFilter* filter;
  const TickStats* ticks;
  const std::atomic<bool>* freeze;
  const std::atomic<bool>* recovering;
  const std::atomic<uint64_t>* reflex_count;
};

inline LinkStats collect_link_stats(const LinkSources& s, int64_t now_ms, uint64_t pkts_last_s) {
  LinkStats l;
  l.cmd_age_ms = s.filter->cmd_age_ms(now_ms);
  l.cmd_pkts_last_s = pkts_last_s;
  l.cmd_drop_size = s.filter->drop_size.load(std::memory_order_relaxed);
  l.cmd_drop_allow = s.filter->drop_allow.load(std::memory_order_relaxed);
  l.cmd_drop_latch = s.filter->drop_latch.load(std::memory_order_relaxed);
  l.latched_sender = s.filter->latched_sender();
  l.missed_cycles_total = s.ticks->pub_missed_total.load(std::memory_order_relaxed);
  l.max_consecutive_missed = s.ticks->pub_max_consec.load(std::memory_order_relaxed);
  l.freeze = s.freeze->load() ? 1 : 0;
  l.recovering = s.recovering->load() ? 1 : 0;
  l.tick_over_1p2ms_1s = s.ticks->pub_over_1p2ms_1s.load(std::memory_order_relaxed);
  l.tick_max_us_1s = s.ticks->pub_max_us_1s.load(std::memory_order_relaxed);
  l.reflex_count = s.reflex_count->load(std::memory_order_relaxed);
  return l;
}

// 5 Hz (period_ms = 200): franka_link.txt. cmd_pkts_last_s = accepted packets over the last
// 5 samples (= 1 s).
inline void run_link_writer(const std::atomic<bool>& stop, const LinkSources& s, StateSink& sink,
                            int period_ms = 200) {
  const int ring_n = period_ms > 0 ? (1000 / period_ms > 0 ? 1000 / period_ms : 1) : 5;
  std::vector<uint64_t> ring(static_cast<size_t>(ring_n), 0);
  size_t ri = 0;
  while (!stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    const uint64_t acc = s.filter->accepted.load(std::memory_order_relaxed);
    const uint64_t last_s = acc - ring[ri];
    ring[ri] = acc;
    ri = (ri + 1) % ring.size();
    sink.emit("franka_link.txt", collect_link_stats(s, steady_now_ms(), last_s).format());
  }
}

}  // namespace rt
