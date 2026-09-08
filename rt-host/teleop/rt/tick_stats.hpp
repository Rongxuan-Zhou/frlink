// rt/tick_stats.hpp - callback entry-interval instrumentation for the 1 kHz control loop.
//
// on_tick() is called at the top of every control callback with CLOCK_MONOTONIC ns.
// It does no allocation, no I/O and no locking: plain integer arithmetic plus relaxed
// atomic stores. The 5 Hz link writer reads the pub_* atomics.
#pragma once
#include <atomic>
#include <cmath>
#include <cstdint>

namespace rt {

struct TickStats {
  static constexpr int64_t kNominalNs = 1000000;          // 1 ms cycle
  static constexpr int64_t kMissedThresholdNs = 1500000;  // > 1.5 ms => missed cycle(s)
  static constexpr int64_t kOverThresholdNs = 1200000;    // > 1.2 ms => "over" in 1 s window
  static constexpr int64_t kWindowNs = 1000000000;        // 1 s window

  // RT-thread private state.
  int64_t last_entry_ns = 0;
  int64_t win_start_ns = 0;
  int64_t win_max_ns = 0;
  uint32_t win_over = 0;
  uint32_t consec = 0;
  uint32_t max_consec = 0;
  uint64_t missed_total = 0;

  // Published (read by the link writer).
  std::atomic<uint64_t> pub_missed_total{0};
  std::atomic<uint32_t> pub_max_consec{0};
  std::atomic<uint32_t> pub_over_1p2ms_1s{0};  // count in the LAST COMPLETED 1 s window
  std::atomic<uint32_t> pub_max_us_1s{0};      // max interval (us) in the LAST COMPLETED window

  // interval > 1.5 ms counts round(interval / 1 ms) - 1 missed cycles.
  static uint32_t missed_for_interval(int64_t interval_ns) {
    if (interval_ns <= kMissedThresholdNs) return 0;
    long long r = std::llround(static_cast<double>(interval_ns) / static_cast<double>(kNominalNs));
    return r > 1 ? static_cast<uint32_t>(r - 1) : 0u;
  }

  // Returns the measured interval (0 on the first tick after construction / reset).
  int64_t on_tick(int64_t now_ns) {
    if (last_entry_ns == 0) {
      last_entry_ns = now_ns;
      win_start_ns = now_ns;
      return 0;
    }
    const int64_t dt = now_ns - last_entry_ns;
    last_entry_ns = now_ns;

    const uint32_t m = missed_for_interval(dt);
    if (m > 0) {
      missed_total += m;
      consec += m;
      if (consec > max_consec) max_consec = consec;
    } else {
      consec = 0;
    }
    if (dt > kOverThresholdNs) ++win_over;
    if (dt > win_max_ns) win_max_ns = dt;
    if (now_ns - win_start_ns >= kWindowNs) {
      pub_over_1p2ms_1s.store(win_over, std::memory_order_relaxed);
      pub_max_us_1s.store(static_cast<uint32_t>(win_max_ns / 1000), std::memory_order_relaxed);
      win_over = 0;
      win_max_ns = 0;
      win_start_ns = now_ns;
    }
    pub_missed_total.store(missed_total, std::memory_order_relaxed);
    pub_max_consec.store(max_consec, std::memory_order_relaxed);
    return dt;
  }

  // Call from the (non-running) control thread before robot.control() is re-entered after a
  // reflex so the recovery gap is not counted as missed cycles.
  void reset_interval() {
    last_entry_ns = 0;
    consec = 0;
  }
};

}  // namespace rt
