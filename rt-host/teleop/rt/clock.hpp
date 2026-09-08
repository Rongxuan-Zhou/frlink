// rt/clock.hpp - tiny clock helpers shared by the RT units (header-only).
#pragma once
#include <chrono>
#include <cstdint>
#include <ctime>

namespace rt {

// Same clock/units as the legacy servos use for g_last_recv_ms (steady_clock, ms).
inline int64_t steady_now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

inline int64_t mono_now_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

inline int64_t real_now_ns() {
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

// CLOCK_REALTIME at first call. main() calls this first thing so it means "process start".
inline int64_t process_epoch_ns() {
  static const int64_t epoch = real_now_ns();
  return epoch;
}

}  // namespace rt
