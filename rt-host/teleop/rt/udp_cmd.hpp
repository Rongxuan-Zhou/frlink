// rt/udp_cmd.hpp - command socket (recvfrom + MSG_TRUNC), pure CmdFilter (allow-list, size,
// sender latch) and the shared listener loop used by both servos and the network stub.
#pragma once
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "clock.hpp"
#include "triple_buffer.hpp"

namespace rt {

constexpr int kCmdPoseFloats = 16;
constexpr ssize_t kCmdPoseBytes = kCmdPoseFloats * static_cast<ssize_t>(sizeof(double));  // 128

inline std::string ip_be_to_string(uint32_t ip_be) {
  char buf[INET_ADDRSTRLEN] = {0};
  in_addr a{};
  a.s_addr = ip_be;
  inet_ntop(AF_INET, &a, buf, sizeof(buf));
  return buf;
}

// "10.10.0.1,127.0.0.1" -> network-byte-order IPv4 list. Returns false on a bad token.
inline bool parse_allow_list(const std::string& csv, std::vector<uint32_t>* out, std::string* err) {
  out->clear();
  size_t start = 0;
  while (start <= csv.size()) {
    size_t comma = csv.find(',', start);
    std::string tok = csv.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
    if (!tok.empty()) {
      in_addr a{};
      if (inet_pton(AF_INET, tok.c_str(), &a) != 1) {
        if (err) *err = "bad --cmd-allow ip '" + tok + "'";
        return false;
      }
      out->push_back(a.s_addr);
    }
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  if (out->empty()) {
    if (err) *err = "--cmd-allow list is empty";
    return false;
  }
  return true;
}

// Pure, testable. Order of checks: allow-list -> size -> latch.
// Latch: first accepted (ip,port) is latched; other sources are dropped (cmd_drop_latch) until
// latch_release_ms elapse without an accepted packet from the latched sender.
class CmdFilter {
 public:
  enum class Verdict { kAccept, kDropAllow, kDropSize, kDropLatch };

  explicit CmdFilter(std::vector<uint32_t> allow_be, ssize_t expect_bytes = kCmdPoseBytes,
                     int64_t latch_release_ms = 1000)
      : allow_(std::move(allow_be)), expect_(expect_bytes), release_ms_(latch_release_ms) {}

  Verdict judge(uint32_t src_ip_be, uint16_t src_port_be, ssize_t n, int64_t now_ms) {
    maybe_release(now_ms);
    bool allowed = false;
    for (uint32_t a : allow_) {
      if (a == src_ip_be) {
        allowed = true;
        break;
      }
    }
    if (!allowed) {
      drop_allow.fetch_add(1, std::memory_order_relaxed);
      return Verdict::kDropAllow;
    }
    if (n != expect_) {
      drop_size.fetch_add(1, std::memory_order_relaxed);
      return Verdict::kDropSize;
    }
    if (latched_.load(std::memory_order_relaxed)) {
      if (src_ip_be != l_ip_.load(std::memory_order_relaxed) ||
          src_port_be != l_port_.load(std::memory_order_relaxed)) {
        drop_latch.fetch_add(1, std::memory_order_relaxed);
        return Verdict::kDropLatch;
      }
    } else {
      l_ip_.store(src_ip_be, std::memory_order_relaxed);
      l_port_.store(src_port_be, std::memory_order_relaxed);
      latched_.store(true, std::memory_order_release);
    }
    last_latched_ms_ = now_ms;
    last_accept_ms.store(now_ms, std::memory_order_relaxed);
    accepted.fetch_add(1, std::memory_order_relaxed);
    return Verdict::kAccept;
  }

  bool accept(uint32_t src_ip_be, uint16_t src_port_be, ssize_t n, int64_t now_ms) {
    return judge(src_ip_be, src_port_be, n, now_ms) == Verdict::kAccept;
  }

  // Also called by the listener on every recv timeout so the latch releases during silence.
  void maybe_release(int64_t now_ms) {
    if (latched_.load(std::memory_order_relaxed) && now_ms - last_latched_ms_ >= release_ms_) {
      latched_.store(false, std::memory_order_release);
    }
  }

  bool latched() const { return latched_.load(std::memory_order_acquire); }

  // "ip:port" or "none". Safe to call from another thread.
  std::string latched_sender() const {
    if (!latched_.load(std::memory_order_acquire)) return "none";
    return ip_be_to_string(l_ip_.load(std::memory_order_relaxed)) + ":" +
           std::to_string(ntohs(l_port_.load(std::memory_order_relaxed)));
  }

  // -1 before the first accepted packet, else now - last accepted (unclamped, keeps growing).
  int64_t cmd_age_ms(int64_t now_ms) const {
    int64_t t = last_accept_ms.load(std::memory_order_relaxed);
    return t == 0 ? -1 : now_ms - t;
  }

  std::atomic<uint64_t> accepted{0};
  std::atomic<uint64_t> drop_size{0};
  std::atomic<uint64_t> drop_allow{0};
  std::atomic<uint64_t> drop_latch{0};
  std::atomic<int64_t> last_accept_ms{0};

 private:
  std::vector<uint32_t> allow_;
  ssize_t expect_;
  int64_t release_ms_;
  std::atomic<bool> latched_{false};
  std::atomic<uint32_t> l_ip_{0};
  std::atomic<uint16_t> l_port_{0};
  int64_t last_latched_ms_ = 0;  // listener-thread private
};

class CommandSocket {
 public:
  CommandSocket() = default;
  ~CommandSocket() { close(); }
  CommandSocket(const CommandSocket&) = delete;
  CommandSocket& operator=(const CommandSocket&) = delete;

  // SO_REUSEADDR + SO_RCVTIMEO 100 ms, like the legacy listener. bind_ip may be "0.0.0.0".
  bool open(const std::string& bind_ip, int port, std::string* err) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
      if (err) *err = std::string("udp socket() failed: ") + strerror(errno);
      return false;
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, bind_ip.c_str(), &addr.sin_addr) != 1) {
      if (err) *err = "bad --cmd-bind ip '" + bind_ip + "'";
      ::close(fd);
      return false;
    }
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      if (err) *err = "udp bind() failed on " + bind_ip + ":" + std::to_string(port) + ": " + strerror(errno);
      ::close(fd);
      return false;
    }
    timeval tv{0, 100 * 1000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    close();
    fd_ = fd;
    return true;
  }

  // Returns the TRUE datagram length (MSG_TRUNC), 0/-1 on timeout or error; fills the source.
  ssize_t recv_from(void* buf, size_t cap, uint32_t* src_ip_be, uint16_t* src_port_be) {
    sockaddr_in src{};
    socklen_t sl = sizeof(src);
    ssize_t n = ::recvfrom(fd_, buf, cap, MSG_TRUNC, reinterpret_cast<sockaddr*>(&src), &sl);
    if (n > 0) {
      *src_ip_be = src.sin_addr.s_addr;
      *src_port_be = src.sin_port;
    }
    return n;
  }

  // Port actually bound (useful when opened with port 0 in tests).
  int local_port() const {
    sockaddr_in a{};
    socklen_t l = sizeof(a);
    if (fd_ < 0 || getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &l) != 0) return -1;
    return ntohs(a.sin_port);
  }

  int fd() const { return fd_; }
  void close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }

 private:
  int fd_ = -1;
};

// Globals of the hosting program that the listener updates (legacy semantics preserved).
struct CmdListenerHooks {
  std::atomic<bool>* stop;
  std::atomic<bool>* freeze;            // packets accepted by the filter but dropped while frozen
  std::atomic<uint64_t>* freeze_drops;
  std::atomic<uint64_t>* packet_count;  // incremented only for packets that reach target
  std::atomic<int64_t>* last_recv_ms;   // steady_now_ms() of the last packet that reached target
};

// Drop-in replacement for the legacy udp_listener() body. The socket must already be open.
inline void run_cmd_listener(CommandSocket& sock, CmdFilter& filter,
                             TripleBuffer<std::array<double, 16>>& target,
                             const CmdListenerHooks& h) {
  alignas(8) unsigned char buf[512];
  while (!h.stop->load()) {
    uint32_t ip = 0;
    uint16_t port = 0;
    ssize_t n = sock.recv_from(buf, sizeof(buf), &ip, &port);
    const int64_t now = steady_now_ms();
    if (n <= 0) {  // timeout (EAGAIN) or error: keep polling, let the latch expire
      filter.maybe_release(now);
      continue;
    }
    if (!filter.accept(ip, port, n, now)) continue;
    if (h.freeze->load()) {
      h.freeze_drops->fetch_add(1);
      continue;
    }
    std::array<double, 16> pose{};
    std::memcpy(pose.data(), buf, static_cast<size_t>(kCmdPoseBytes));
    target.write(pose);
    h.packet_count->fetch_add(1, std::memory_order_release);
    h.last_recv_ms->store(now);
  }
  sock.close();
}

}  // namespace rt
