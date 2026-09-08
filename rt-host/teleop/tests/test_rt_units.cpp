// tests/test_rt_units.cpp - assert-based unit tests for teleop/rt/*.hpp (no libfranka needed).
// Each unit's tests live in tests/rt_units/<unit>_tests.inc and are compiled in when present,
// so the binary grows section by section (tasks B1..B5) without editing this file.
//
// Build + run (inside the PART A image):
//   FRANKA_RUN_AS_USER=1 /home/rongxuan_zhou/franka/bin/franka-run bash -c
//     'cd /franka/teleop && g++ -O2 -std=c++17 -pthread tests/test_rt_units.cpp -o tests/test_rt_units && tests/test_rt_units'
#include <cstdio>
#include <cstdlib>
#include <string>

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond)                                                                     \
  do {                                                                                  \
    ++g_checks;                                                                         \
    if (!(cond)) {                                                                      \
      ++g_failures;                                                                     \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);             \
    }                                                                                   \
  } while (0)

#define CHECK_EQ_STR(a, b)                                                              \
  do {                                                                                  \
    ++g_checks;                                                                         \
    std::string _a = (a), _b = (b);                                                     \
    if (_a != _b) {                                                                     \
      ++g_failures;                                                                     \
      std::fprintf(stderr, "FAIL %s:%d: %s\n   got:  \"%s\"\n   want: \"%s\"\n",       \
                   __FILE__, __LINE__, #a " == " #b, _a.c_str(), _b.c_str());           \
    }                                                                                   \
  } while (0)

#define SECTION(name) std::printf("[%s]\n", name)

#if __has_include("rt_units/triple_buffer_tests.inc")
#include "rt_units/triple_buffer_tests.inc"
#define HAVE_TRIPLE_BUFFER_TESTS 1
#endif
#if __has_include("rt_units/tick_stats_tests.inc")
#include "rt_units/tick_stats_tests.inc"
#define HAVE_TICK_STATS_TESTS 1
#endif
#if __has_include("rt_units/state_pub_tests.inc")
#include "rt_units/state_pub_tests.inc"
#define HAVE_STATE_PUB_TESTS 1
#endif
#if __has_include("rt_units/udp_cmd_tests.inc")
#include "rt_units/udp_cmd_tests.inc"
#define HAVE_UDP_CMD_TESTS 1
#endif
#if __has_include("rt_units/rt_setup_cli_tests.inc")
#include "rt_units/rt_setup_cli_tests.inc"
#define HAVE_RT_SETUP_CLI_TESTS 1
#endif
#if __has_include("rt_units/writers_tests.inc")
#include "rt_units/writers_tests.inc"
#define HAVE_WRITERS_TESTS 1
#endif

int main() {
#ifdef HAVE_TRIPLE_BUFFER_TESTS
  test_triple_buffer();
#endif
#ifdef HAVE_TICK_STATS_TESTS
  test_tick_stats();
#endif
#ifdef HAVE_STATE_PUB_TESTS
  test_state_pub();
#endif
#ifdef HAVE_UDP_CMD_TESTS
  test_udp_cmd();
#endif
#ifdef HAVE_RT_SETUP_CLI_TESTS
  test_rt_setup_cli();
#endif
#ifdef HAVE_WRITERS_TESTS
  test_writers();
#endif
  std::printf("%s: %d checks, %d failures\n", g_failures ? "FAILED" : "OK", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
