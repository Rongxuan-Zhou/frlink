// tests/rt_units/tick_stats_tests.inc - included by tests/test_rt_units.cpp
#include "../../rt/tick_stats.hpp"

static void test_tick_stats() {
  SECTION("tick_stats");
  using rt::TickStats;
  CHECK(TickStats::missed_for_interval(1000000) == 0);
  CHECK(TickStats::missed_for_interval(1400000) == 0);
  CHECK(TickStats::missed_for_interval(1500000) == 0);   // threshold is strictly greater
  CHECK(TickStats::missed_for_interval(1600000) == 1);   // round(1.6) - 1
  CHECK(TickStats::missed_for_interval(2000000) == 1);
  CHECK(TickStats::missed_for_interval(2400000) == 1);
  CHECK(TickStats::missed_for_interval(3100000) == 2);
  CHECK(TickStats::missed_for_interval(10000000) == 9);

  TickStats ts;
  int64_t t = 5000000000LL;  // arbitrary monotonic origin
  CHECK(ts.on_tick(t) == 0);  // first tick: no interval
  t += 1000000; CHECK(ts.on_tick(t) == 1000000);
  CHECK(ts.pub_missed_total.load() == 0);
  t += 1400000; ts.on_tick(t);           // over 1.2 ms, not missed
  CHECK(ts.pub_missed_total.load() == 0);
  CHECK(ts.win_over == 1);
  t += 2000000; ts.on_tick(t);           // 1 missed
  t += 2000000; ts.on_tick(t);           // 1 missed -> consec 2
  CHECK(ts.pub_missed_total.load() == 2);
  CHECK(ts.pub_max_consec.load() == 2);
  t += 1000000; ts.on_tick(t);           // resets consec
  CHECK(ts.consec == 0);
  t += 3100000; ts.on_tick(t);           // 2 missed, consec 2 (max stays 2)
  CHECK(ts.pub_missed_total.load() == 4);
  CHECK(ts.pub_max_consec.load() == 2);
  t += 4000000; ts.on_tick(t);           // 3 missed, consec 5
  CHECK(ts.pub_max_consec.load() == 5);
  CHECK(ts.win_over == 5);               // 1.4, 2.0, 2.0, 3.1, 4.0
  CHECK(ts.win_max_ns == 4000000);
  // window not yet complete -> published window values still 0
  CHECK(ts.pub_over_1p2ms_1s.load() == 0);
  CHECK(ts.pub_max_us_1s.load() == 0);
  // advance to 1 s with clean ticks -> window closes
  {
    const int64_t win_end = ts.win_start_ns + TickStats::kWindowNs;
    while (t < win_end) { t += 1000000; ts.on_tick(t); }
  }
  CHECK(ts.pub_over_1p2ms_1s.load() == 5);
  CHECK(ts.pub_max_us_1s.load() == 4000);
  CHECK(ts.win_over == 0);
  CHECK(ts.win_max_ns == 0);
  // next window: one 1.3 ms tick then close
  t += 1300000; ts.on_tick(t);
  {
    const int64_t win_end = ts.win_start_ns + TickStats::kWindowNs;
    while (t < win_end) { t += 1000000; ts.on_tick(t); }
  }
  CHECK(ts.pub_over_1p2ms_1s.load() == 1);
  CHECK(ts.pub_max_us_1s.load() == 1300);
  // reset_interval: a 500 ms recovery gap must not count
  uint64_t before = ts.pub_missed_total.load();
  ts.reset_interval();
  t += 500000000; CHECK(ts.on_tick(t) == 0);
  t += 1000000; ts.on_tick(t);
  CHECK(ts.pub_missed_total.load() == before);
}
