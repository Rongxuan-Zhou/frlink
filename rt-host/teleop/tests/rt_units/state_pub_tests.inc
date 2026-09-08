// tests/rt_units/state_pub_tests.inc - included by tests/test_rt_units.cpp
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <sstream>

#include "../../rt/state_pub.hpp"

static void test_state_pub() {
  SECTION("state_pub");
  // format_doubles byte-identity vs the legacy inline loop under default ostream flags
  const double tricky[] = {0.0, 1e-7, 123456.7, -0.0001234, 1e6, 0.1 + 0.2, 1.0, -0.0, 3.14159265358979, 1e-320, 2.5e10};
  const int n = sizeof(tricky) / sizeof(tricky[0]);
  {
    std::ostringstream ref;
    for (int i = 0; i < n; ++i) ref << tricky[i] << (i + 1 < n ? ' ' : '\n');
    CHECK_EQ_STR(rt::format_doubles(tricky, n), ref.str());
  }
  CHECK_EQ_STR(rt::format_doubles(tricky, 6), "0 1e-07 123457 -0.0001234 1e+06 0.3\n");
  CHECK_EQ_STR(rt::format_doubles(tricky, 1), "0\n");
  {
    // 16 / 6 / 28 token counts, single trailing newline, no leading/trailing spaces
    double v[28];
    for (int i = 0; i < 28; ++i) v[i] = i * 0.125 - 1.0;
    for (int cnt : {16, 6, 28}) {
      std::string s = rt::format_doubles(v, cnt);
      CHECK(!s.empty() && s.back() == '\n');
      CHECK(s.find('\n') == s.size() - 1);
      int tokens = 0;
      std::istringstream is(s);
      std::string tok;
      while (is >> tok) ++tokens;
      CHECK(tokens == cnt);
      CHECK(s.front() != ' ' && s[s.size() - 2] != ' ');
    }
  }
  // FRST1 header layout
  {
    std::string d = rt::StatePublisher::build_datagram(7, 1000, 2000, 3000, "franka_current_ee.txt", "1 2 3\n");
    CHECK_EQ_STR(d, "FRST1 7 1000 2000 3000 franka_current_ee.txt\n1 2 3\n");
  }
  // LinkStats formatter: key order and defaults
  {
    rt::LinkStats l;
    CHECK_EQ_STR(l.format(),
                 "cmd_age_ms=-1 cmd_pkts_last_s=0 cmd_drop_size=0 cmd_drop_allow=0 cmd_drop_latch=0 "
                 "latched_sender=none missed_cycles_total=0 max_consecutive_missed=0 freeze=0 "
                 "recovering=0 tick_over_1p2ms_1s=0 tick_max_us_1s=0 reflex_count=0\n");
    l.cmd_age_ms = 1234; l.latched_sender = "10.10.0.1:40000"; l.freeze = 1; l.reflex_count = 3;
    CHECK(l.format().find("cmd_age_ms=1234 ") == 0);
    CHECK(l.format().find(" latched_sender=10.10.0.1:40000 ") != std::string::npos);
    CHECK(l.format().find(" freeze=1 ") != std::string::npos);
    CHECK(l.format().find(" reflex_count=3\n") != std::string::npos);
  }
  // parse_host_port
  {
    std::string h; int p = 0;
    CHECK(rt::parse_host_port("10.10.0.1:50002", &h, &p) && h == "10.10.0.1" && p == 50002);
    CHECK(!rt::parse_host_port("10.10.0.1", &h, &p));
    CHECK(!rt::parse_host_port(":5", &h, &p));
    CHECK(!rt::parse_host_port("1.2.3.4:", &h, &p));
    CHECK(!rt::parse_host_port("1.2.3.4:99999", &h, &p));
  }
  // Publisher over loopback: seq starts at 0 and increments per datagram; body is verbatim.
  {
    int rx = socket(AF_INET, SOCK_DGRAM, 0);
    CHECK(rx >= 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = 0;
    CHECK(bind(rx, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0);
    socklen_t al = sizeof(a);
    getsockname(rx, reinterpret_cast<sockaddr*>(&a), &al);
    timeval tv{2, 0};
    setsockopt(rx, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    rt::StatePublisher pub;
    std::string err;
    CHECK(!pub.enabled());
    CHECK(!pub.send("x", "y"));                       // disabled: no send, no seq
    CHECK(pub.next_seq() == 0);
    CHECK(!pub.open("nonsense", &err));
    CHECK(pub.open("127.0.0.1:" + std::to_string(ntohs(a.sin_port)), &err));
    pub.set_epoch_ns(42);
    CHECK(pub.enabled());
    CHECK(pub.send("franka_wrench.txt", "0 0 0 0 0 0\n"));
    CHECK(pub.send("franka_link.txt", "cmd_age_ms=-1\n"));
    CHECK(pub.next_seq() == 2);
    for (int k = 0; k < 2; ++k) {
      char buf[600];
      ssize_t n = recv(rx, buf, sizeof(buf), 0);
      CHECK(n > 0);
      std::string d(buf, n > 0 ? static_cast<size_t>(n) : 0);
      std::istringstream hs(d.substr(0, d.find('\n')));
      std::string magic, name; unsigned long long seq, epoch, treal, tmono;
      hs >> magic >> seq >> epoch >> treal >> tmono >> name;
      CHECK_EQ_STR(magic, "FRST1");
      CHECK(seq == static_cast<unsigned long long>(k));
      CHECK(epoch == 42);
      CHECK(treal > 1600000000000000000ull);            // after 2020 in ns
      CHECK(tmono > 0);
      CHECK_EQ_STR(name, k == 0 ? "franka_wrench.txt" : "franka_link.txt");
      CHECK_EQ_STR(d.substr(d.find('\n') + 1), k == 0 ? "0 0 0 0 0 0\n" : "cmd_age_ms=-1\n");
    }
    // StateSink: file body == datagram body, file truncate-rewritten
    char dir_tpl[] = "/tmp/rt_units_XXXXXX";
    char* dir = mkdtemp(dir_tpl);
    CHECK(dir != nullptr);
    rt::StateSink sink(dir, true, &pub);
    sink.emit("franka_current_ee.txt", "1 2 3\n");
    sink.emit("franka_current_ee.txt", "4 5\n");
    {
      std::ifstream f(std::string(dir) + "/franka_current_ee.txt");
      std::stringstream ss; ss << f.rdbuf();
      CHECK_EQ_STR(ss.str(), "4 5\n");
    }
    char buf[600];
    ssize_t n1 = recv(rx, buf, sizeof(buf), 0);
    ssize_t n2 = recv(rx, buf, sizeof(buf), 0);
    CHECK(n1 > 0 && n2 > 0);
    std::string d2(buf, n2 > 0 ? static_cast<size_t>(n2) : 0);
    CHECK_EQ_STR(d2.substr(d2.find('\n') + 1), "4 5\n");
    CHECK(pub.next_seq() == 4);
    // files disabled: no file, datagram still sent
    rt::StateSink nofile(dir, false, &pub);
    nofile.emit("franka_wrench.txt", "9\n");
    CHECK(!std::ifstream(std::string(dir) + "/franka_wrench.txt").good());
    CHECK(recv(rx, buf, sizeof(buf), 0) > 0);
    // publish_only never writes the file
    nofile.publish_only("franka_init_pose.txt", "1\n");
    CHECK(!std::ifstream(std::string(dir) + "/franka_init_pose.txt").good());
    CHECK(recv(rx, buf, sizeof(buf), 0) > 0);
    unlink((std::string(dir) + "/franka_current_ee.txt").c_str());
    rmdir(dir);
    close(rx);
  }
}
