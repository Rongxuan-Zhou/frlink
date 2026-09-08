// tests/rt_units/rt_setup_cli_tests.inc - included by tests/test_rt_units.cpp
#include <cstdlib>
#include <sstream>
#include <vector>

#include "../../rt/cli.hpp"
#include "../../rt/rt_setup.hpp"

static void test_rt_setup_cli() {
  SECTION("rt_setup");
  std::vector<int> cpus;
  std::string err;
  CHECK(rt::parse_cpu_list("4-15", &cpus, &err) && cpus.size() == 12 && cpus.front() == 4 && cpus.back() == 15);
  CHECK(rt::parse_cpu_list("4,5,6", &cpus, &err) && cpus == std::vector<int>({4, 5, 6}));
  CHECK(rt::parse_cpu_list("6,4-5,4", &cpus, &err) && cpus == std::vector<int>({4, 5, 6}));
  CHECK(rt::parse_cpu_list("2", &cpus, &err) && cpus == std::vector<int>({2}));
  CHECK(!rt::parse_cpu_list("", &cpus, &err));
  CHECK(!rt::parse_cpu_list("4-", &cpus, &err));
  CHECK(!rt::parse_cpu_list("5-4", &cpus, &err));
  CHECK(!rt::parse_cpu_list("a", &cpus, &err));
  CHECK(!rt::parse_cpu_list("1,,2", &cpus, &err));
  CHECK_EQ_STR(rt::cpu_list_to_string({4, 5, 6, 7, 10}), "4-7,10");
  CHECK_EQ_STR(rt::cpu_list_to_string({2}), "2");
  CHECK_EQ_STR(rt::cpu_list_to_string({}), "");
  // prefault: never more than requested, never above the stack limit
  size_t pf = rt::prefault_stack(1u << 20);
  CHECK(pf == (1u << 20));
  // memlock guard: with a finite RLIMIT_MEMLOCK, lock_all_memory must refuse (no MCL_FUTURE) and explain
  {
    rlimit saved{};
    CHECK(getrlimit(RLIMIT_MEMLOCK, &saved) == 0);
    rlimit low = saved;
    low.rlim_cur = 1u << 20;   // 1 MB soft limit (always <= hard)
    CHECK(setrlimit(RLIMIT_MEMLOCK, &low) == 0);
    std::string merr;
    CHECK(!rt::lock_all_memory(&merr));
    CHECK(merr.find("skipped mlockall") != std::string::npos);
    CHECK(setrlimit(RLIMIT_MEMLOCK, &saved) == 0);
  }
  // affinity round trip on cpu 0 (exists everywhere), then restore
  std::vector<int> before = rt::self_affinity();
  CHECK(!before.empty());
  CHECK(rt::pin_self(0, &err));
  CHECK(rt::self_affinity() == std::vector<int>({0}));
  CHECK(rt::set_self_affinity(before, &err));
  CHECK(rt::self_affinity() == before);
  CHECK(!rt::set_self_affinity({CPU_SETSIZE - 1}, &err));  // nonexistent cpu -> EINVAL
  // check-rt report runs and mentions the probe threads
  std::ostringstream rep;
  CHECK(rt::run_check_rt(rep, {}, -1) == 0);
  CHECK(rep.str().find("mlockall:") != std::string::npos);
  CHECK(rep.str().find("probe_helper1") != std::string::npos);
  CHECK(rep.str().find("check-rt done") != std::string::npos);
  std::printf("%s", rep.str().c_str());

  SECTION("cli");
  {
    const char* argv[] = {"servo", "172.16.0.2", "--load-mass", "0.0", "--cmd-bind", "10.10.0.2",
                          "--no-jlim", "--cmd-allow", "10.10.0.1,127.0.0.1", "--state-dst", "10.10.0.1:50002",
                          "--kz", "3000", "--rt-cpu", "2", "--aux-cpus", "4-15", "--state-files", "1",
                          "--state-dir", "/tmp/franka", "--cmd-port", "50005", "--null-mode", "custom"};
    int argc = sizeof(argv) / sizeof(argv[0]);
    rt::RtCliOptions o;
    CHECK(rt::prescan_rt_args(argc, const_cast<char**>(argv), &o, &err));
    CHECK_EQ_STR(o.cmd_bind, "10.10.0.2");
    CHECK_EQ_STR(o.cmd_allow, "10.10.0.1,127.0.0.1");
    CHECK_EQ_STR(o.state_dst, "10.10.0.1:50002");
    CHECK_EQ_STR(o.state_dir, "/tmp/franka");
    CHECK(o.state_files && o.rt_cpu == 2 && !o.help && !o.check_rt);
    CHECK_EQ_STR(o.aux_cpus, "4-15");
    // legacy argv survives in order; --cmd-port became a trailing --port
    std::vector<std::string> want = {"servo", "172.16.0.2", "--load-mass", "0.0", "--no-jlim", "--kz", "3000",
                                     "--null-mode", "custom", "--port", "50005"};
    CHECK(o.rest_storage == want);
    CHECK(o.rest_argc() == static_cast<int>(want.size()));
    CHECK_EQ_STR(o.rest_argv_ptr()[1], "172.16.0.2");
    // run the exact legacy loop over the stripped argv
    int port = 50001; double kz = -1; bool no_jlim = false; std::string null_mode = "init";
    int argc2 = o.rest_argc(); char** argv2 = o.rest_argv_ptr();
    for (int i = 2; i < argc2; i += 2) {
      if (i + 1 >= argc2) break;
      std::string flag = argv2[i];
      if (flag == "--port") port = std::stoi(argv2[i + 1]);
      else if (flag == "--kz") kz = std::stod(argv2[i + 1]);
      else if (flag == "--no-jlim") { no_jlim = true; i--; }
      else if (flag == "--null-mode") null_mode = argv2[i + 1];
    }
    CHECK(port == 50005 && kz == 3000 && no_jlim && null_mode == "custom");
  }
  {
    const char* argv[] = {"servo", "172.16.0.2", "--port", "50003"};
    rt::RtCliOptions o;
    setenv("FRANKA_STATE_DIR", "/tmp/franka_env", 1);
    CHECK(rt::prescan_rt_args(4, const_cast<char**>(argv), &o, &err));
    CHECK_EQ_STR(o.state_dir, "/tmp/franka_env");
    CHECK(o.rest_storage == std::vector<std::string>({"servo", "172.16.0.2", "--port", "50003"}));
    CHECK_EQ_STR(o.cmd_bind, "127.0.0.1");
    CHECK_EQ_STR(o.cmd_allow, "127.0.0.1");
    CHECK(o.state_dst.empty() && o.aux_cpus.empty() && o.rt_cpu == -1 && o.state_files);
    unsetenv("FRANKA_STATE_DIR");
    rt::RtCliOptions o2;
    CHECK(rt::prescan_rt_args(4, const_cast<char**>(argv), &o2, &err));
    CHECK_EQ_STR(o2.state_dir, "/tmp");
  }
  {
    const char* argv[] = {"servo", "--check-rt", "--rt-cpu", "2", "--aux-cpus", "4-15"};
    rt::RtCliOptions o;
    CHECK(rt::prescan_rt_args(6, const_cast<char**>(argv), &o, &err));
    CHECK(o.check_rt && o.rt_cpu == 2 && o.rest_argc() == 1);
    const char* h[] = {"servo", "--help"};
    rt::RtCliOptions oh;
    CHECK(rt::prescan_rt_args(2, const_cast<char**>(h), &oh, &err) && oh.help);
    const char* bad[] = {"servo", "1.2.3.4", "--rt-cpu"};
    rt::RtCliOptions ob;
    CHECK(!rt::prescan_rt_args(3, const_cast<char**>(bad), &ob, &err));
    const char* bad2[] = {"servo", "1.2.3.4", "--state-files", "2"};
    CHECK(!rt::prescan_rt_args(4, const_cast<char**>(bad2), &ob, &err));
    const char* bad3[] = {"servo", "1.2.3.4", "--rt-cpu", "x"};
    CHECK(!rt::prescan_rt_args(4, const_cast<char**>(bad3), &ob, &err));
  }
  CHECK(rt::rt_flags_help().find("--cmd-allow") != std::string::npos);
}
