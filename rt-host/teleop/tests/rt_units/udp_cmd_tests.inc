// tests/rt_units/udp_cmd_tests.inc - included by tests/test_rt_units.cpp
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <thread>
#include <vector>

#include "../../rt/udp_cmd.hpp"

static uint32_t ip_be(const char* s) {
  in_addr a{};
  inet_pton(AF_INET, s, &a);
  return a.s_addr;
}

static void test_udp_cmd() {
  SECTION("udp_cmd");
  using V = rt::CmdFilter::Verdict;
  std::vector<uint32_t> allow;
  std::string err;
  CHECK(rt::parse_allow_list("10.10.0.1,127.0.0.1", &allow, &err) && allow.size() == 2);
  CHECK(allow[0] == ip_be("10.10.0.1") && allow[1] == ip_be("127.0.0.1"));
  CHECK(!rt::parse_allow_list("10.10.0.x", &allow, &err));
  CHECK(!rt::parse_allow_list("", &allow, &err));
  CHECK(rt::parse_allow_list("10.10.0.1", &allow, &err) && allow.size() == 1);
  CHECK_EQ_STR(rt::ip_be_to_string(ip_be("10.10.0.1")), "10.10.0.1");

  const uint32_t A = ip_be("10.10.0.1"), B = ip_be("10.10.0.9"), L = ip_be("127.0.0.1");
  const uint16_t p1 = htons(40000), p2 = htons(40001);
  rt::CmdFilter f({A, L});  // 128-byte datagrams, 1000 ms latch release
  CHECK(!f.latched());
  CHECK_EQ_STR(f.latched_sender(), "none");
  CHECK(f.cmd_age_ms(500) == -1);

  CHECK(f.judge(B, p1, 128, 100) == V::kDropAllow);            // not in allow list
  CHECK(f.drop_allow.load() == 1 && !f.latched());
  CHECK(f.judge(A, p1, 100, 100) == V::kDropSize);              // wrong size, no latch yet
  CHECK(f.drop_size.load() == 1 && !f.latched());
  CHECK(f.judge(A, p1, 128, 100) == V::kAccept);                // first accepted -> latched
  CHECK(f.latched());
  CHECK_EQ_STR(f.latched_sender(), "10.10.0.1:40000");
  CHECK(f.accepted.load() == 1);
  CHECK(f.cmd_age_ms(350) == 250);
  CHECK(f.judge(A, p2, 128, 200) == V::kDropLatch);             // same ip, other port
  CHECK(f.judge(L, p1, 128, 200) == V::kDropLatch);             // other allowed ip
  CHECK(f.drop_latch.load() == 2);
  CHECK(f.judge(A, p1, 300, 200) == V::kDropSize);              // wrong size from latched sender
  CHECK(f.judge(A, p1, 128, 500) == V::kAccept);                // refreshes the latch (t=500)
  CHECK(f.judge(L, p1, 128, 1499) == V::kDropLatch);            // 999 ms after last -> still latched
  CHECK(f.judge(L, p1, 128, 1500) == V::kAccept);               // 1000 ms -> released, re-latched to L
  CHECK_EQ_STR(f.latched_sender(), "127.0.0.1:40000");
  CHECK(f.judge(A, p1, 128, 1600) == V::kDropLatch);
  // silent release via maybe_release (what the listener calls on recv timeout)
  f.maybe_release(2499);
  CHECK(f.latched());
  f.maybe_release(2500);
  CHECK(!f.latched());
  CHECK_EQ_STR(f.latched_sender(), "none");
  CHECK(f.cmd_age_ms(4000) == 2500);                           // age keeps growing, unclamped
  CHECK(f.accept(A, p2, 128, 2600));
  CHECK_EQ_STR(f.latched_sender(), "10.10.0.1:40001");
  CHECK(f.accepted.load() == 4 && f.drop_size.load() == 2 && f.drop_allow.load() == 1 && f.drop_latch.load() == 4);

  // CommandSocket round trip on loopback, true length via MSG_TRUNC, bad bind ip rejected
  rt::CommandSocket cs;
  CHECK(!cs.open("999.1.1.1", 0, &err));
  CHECK(cs.open("127.0.0.1", 0, &err));
  int port = cs.local_port();
  CHECK(port > 0);
  int tx = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  dst.sin_port = htons(static_cast<uint16_t>(port));
  double pose[16];
  for (int i = 0; i < 16; ++i) pose[i] = i + 0.5;
  CHECK(sendto(tx, pose, sizeof(pose), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst)) == 128);
  unsigned char big[300] = {0};
  CHECK(sendto(tx, big, sizeof(big), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst)) == 300);
  unsigned char buf[512];
  uint32_t sip = 0; uint16_t sport = 0;
  ssize_t n = cs.recv_from(buf, sizeof(buf), &sip, &sport);
  CHECK(n == 128);
  CHECK(sip == ip_be("127.0.0.1"));
  CHECK(sport != 0);
  double got[16];
  std::memcpy(got, buf, 128);
  CHECK(got[0] == 0.5 && got[15] == 15.5);
  n = cs.recv_from(buf, 128, &sip, &sport);                     // cap 128, datagram 300 -> 300 reported
  CHECK(n == 300);
  n = cs.recv_from(buf, sizeof(buf), &sip, &sport);             // 100 ms timeout -> <= 0
  CHECK(n <= 0);

  // run_cmd_listener end-to-end with the freeze hook
  std::atomic<bool> stop{false}, freeze{false};
  std::atomic<uint64_t> freeze_drops{0}, packet_count{0};
  std::atomic<int64_t> last_recv_ms{0};
  rt::CmdListenerHooks hooks{&stop, &freeze, &freeze_drops, &packet_count, &last_recv_ms};
  rt::CmdFilter lf({ip_be("127.0.0.1")});
  rt::TripleBuffer<std::array<double, 16>> target;
  std::thread th([&] { rt::run_cmd_listener(cs, lf, target, hooks); });
  auto wait_for = [](std::atomic<uint64_t>& c, uint64_t v) {
    for (int i = 0; i < 200 && c.load() < v; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    return c.load() >= v;
  };
  pose[12] = 0.25;
  sendto(tx, pose, sizeof(pose), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
  CHECK(wait_for(packet_count, 1));
  CHECK(target.read()[12] == 0.25);
  CHECK(last_recv_ms.load() > 0);
  CHECK(lf.latched());
  freeze.store(true);
  sendto(tx, pose, sizeof(pose), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
  CHECK(wait_for(freeze_drops, 1));
  CHECK(packet_count.load() == 1);
  CHECK(lf.accepted.load() == 2);                               // filter-level accept counts frozen packets
  sendto(tx, big, 77, 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
  CHECK(wait_for(lf.drop_size, 1));
  stop.store(true);
  th.join();
  CHECK(cs.fd() < 0);                                           // listener closed the socket
  close(tx);
}
