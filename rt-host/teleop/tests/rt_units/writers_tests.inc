// tests/rt_units/writers_tests.inc - included by tests/test_rt_units.cpp
#include <fstream>
#include <sstream>
#include <thread>

#include "../../rt/writers.hpp"

static std::string slurp(const std::string& path) {
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

static void test_writers() {
  SECTION("writers");
  char dir_tpl[] = "/tmp/rt_writers_XXXXXX";
  char* dir = mkdtemp(dir_tpl);
  CHECK(dir != nullptr);
  rt::StateSink sink(dir, true, nullptr);
  std::atomic<bool> stop{false}, freeze{false}, recovering{false};
  std::atomic<uint64_t> reflex{0};
  rt::CmdFilter filter({0x0100007f});
  rt::TickStats ticks;
  rt::LinkSources src{&filter, &ticks, &freeze, &recovering, &reflex};
  // link stats collector: -1 age before any packet, values pass through
  {
    rt::LinkStats l = rt::collect_link_stats(src, 1000, 0);
    CHECK(l.cmd_age_ms == -1);
    filter.accept(0x0100007f, htons(1), 128, 700);
    freeze.store(true); reflex.store(2);
    ticks.on_tick(1000000000LL); ticks.on_tick(1003000000LL);  // 3 ms gap -> 2 missed
    l = rt::collect_link_stats(src, 1000, 7);
    CHECK(l.cmd_age_ms == 300 && l.cmd_pkts_last_s == 7 && l.freeze == 1 && l.reflex_count == 2);
    CHECK(l.missed_cycles_total == 2 && l.max_consecutive_missed == 2);
    CHECK_EQ_STR(l.latched_sender, "127.0.0.1:1");
    freeze.store(false);
  }
  // periodic emitter + link writer produce the files with the right token counts
  rt::TripleBuffer<std::array<double, 6>> wrench;
  std::array<double, 6> w{1, 2, 3, 4, 5, 6.5};
  wrench.write(w);
  std::thread tw([&] { rt::run_periodic_emit<6>(stop, 20, "franka_wrench.txt", wrench, sink); });
  std::thread tl([&] { rt::run_link_writer(stop, src, sink, 20); });
  std::thread ti([&] { rt::run_init_pose_resender(stop, 30, "0 0 0\n", sink); });
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  stop.store(true);
  tw.join(); tl.join(); ti.join();
  CHECK_EQ_STR(slurp(std::string(dir) + "/franka_wrench.txt"), "1 2 3 4 5 6.5\n");
  std::string link = slurp(std::string(dir) + "/franka_link.txt");
  CHECK(link.find("cmd_age_ms=") == 0);
  int keys = 0;
  std::istringstream is(link);
  std::string tok;
  while (is >> tok) { ++keys; CHECK(tok.find('=') != std::string::npos); }
  CHECK(keys == 13);
  CHECK(link.find(" cmd_pkts_last_s=1 ") != std::string::npos);
  CHECK(!std::ifstream(std::string(dir) + "/franka_init_pose.txt").good());  // resender is datagram-only
  unlink((std::string(dir) + "/franka_wrench.txt").c_str());
  unlink((std::string(dir) + "/franka_link.txt").c_str());
  rmdir(dir);
}
