// rt/state_pub.hpp - state file body formatting + UDP state publisher (FRST1) + StateSink.
//
// ONE formatter (format_doubles / LinkStats::format) produces the body string; StateSink
// writes that same string to <dir>/<name> (truncate-rewrite, like the legacy writers) and,
// if a --state-dst publisher is open, sends it as one UDP datagram:
//     "FRST1 <seq> <epoch_ns> <t_real_ns> <t_mono_ns> <name>\n" + body
#pragma once
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

#include "clock.hpp"

namespace rt {

// Byte-identical to the legacy `f << v[i] << (i + 1 < n ? ' ' : '\n')` loop under default
// ostream flags (defaultfloat, precision 6).
inline std::string format_doubles(const double* v, int n) {
  std::ostringstream os;
  for (int i = 0; i < n; ++i) os << v[i] << (i + 1 < n ? ' ' : '\n');
  return os.str();
}

struct LinkStats {
  int64_t cmd_age_ms = -1;
  uint64_t cmd_pkts_last_s = 0;
  uint64_t cmd_drop_size = 0;
  uint64_t cmd_drop_allow = 0;
  uint64_t cmd_drop_latch = 0;
  std::string latched_sender = "none";
  uint64_t missed_cycles_total = 0;
  uint32_t max_consecutive_missed = 0;
  int freeze = 0;
  int recovering = 0;
  uint32_t tick_over_1p2ms_1s = 0;
  uint32_t tick_max_us_1s = 0;
  uint64_t reflex_count = 0;

  std::string format() const {
    std::ostringstream os;
    os << "cmd_age_ms=" << cmd_age_ms
       << " cmd_pkts_last_s=" << cmd_pkts_last_s
       << " cmd_drop_size=" << cmd_drop_size
       << " cmd_drop_allow=" << cmd_drop_allow
       << " cmd_drop_latch=" << cmd_drop_latch
       << " latched_sender=" << latched_sender
       << " missed_cycles_total=" << missed_cycles_total
       << " max_consecutive_missed=" << max_consecutive_missed
       << " freeze=" << freeze
       << " recovering=" << recovering
       << " tick_over_1p2ms_1s=" << tick_over_1p2ms_1s
       << " tick_max_us_1s=" << tick_max_us_1s
       << " reflex_count=" << reflex_count << '\n';
    return os.str();
  }
};

// "10.10.0.1:50002" -> host, port. Returns false on malformed input.
inline bool parse_host_port(const std::string& s, std::string* host, int* port) {
  auto c = s.rfind(':');
  if (c == std::string::npos || c == 0 || c + 1 >= s.size()) return false;
  *host = s.substr(0, c);
  try {
    *port = std::stoi(s.substr(c + 1));
  } catch (...) {
    return false;
  }
  return *port > 0 && *port < 65536;
}

class StatePublisher {
 public:
  StatePublisher() = default;
  ~StatePublisher() { close(); }
  StatePublisher(const StatePublisher&) = delete;
  StatePublisher& operator=(const StatePublisher&) = delete;

  bool open(const std::string& host_port, std::string* err) {
    std::string host;
    int port = 0;
    if (!parse_host_port(host_port, &host, &port)) {
      if (err) *err = "bad --state-dst '" + host_port + "' (want ip:port)";
      return false;
    }
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &dst.sin_addr) != 1) {
      if (err) *err = "bad --state-dst ip '" + host + "'";
      return false;
    }
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
      if (err) *err = std::string("socket(): ") + strerror(errno);
      return false;
    }
    close();
    fd_ = fd;
    dst_ = dst;
    epoch_ns_ = process_epoch_ns();
    return true;
  }

  void close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }
  bool enabled() const { return fd_ >= 0; }
  void set_epoch_ns(int64_t e) { epoch_ns_ = e; }
  int64_t epoch_ns() const { return epoch_ns_; }
  uint64_t next_seq() const { return seq_.load(std::memory_order_relaxed); }

  static std::string build_datagram(uint64_t seq, int64_t epoch_ns, int64_t t_real_ns,
                                    int64_t t_mono_ns, const std::string& name,
                                    const std::string& body) {
    std::string d;
    d.reserve(96 + name.size() + body.size());
    d += "FRST1 ";
    d += std::to_string(seq);
    d += ' ';
    d += std::to_string(epoch_ns);
    d += ' ';
    d += std::to_string(t_real_ns);
    d += ' ';
    d += std::to_string(t_mono_ns);
    d += ' ';
    d += name;
    d += '\n';
    d += body;
    return d;
  }

  // Thread-safe. Serialized under send_mu_ so that seq order == wire order across the writer
  // threads (helpers only; the RT tick never calls send). Returns false if disabled/failed.
  bool send(const std::string& name, const std::string& body) {
    if (fd_ < 0) return false;
    std::lock_guard<std::mutex> lk(send_mu_);
    const uint64_t seq = seq_.fetch_add(1, std::memory_order_relaxed);
    const std::string d = build_datagram(seq, epoch_ns_, real_now_ns(), mono_now_ns(), name, body);
    ssize_t n = ::sendto(fd_, d.data(), d.size(), MSG_DONTWAIT,
                         reinterpret_cast<const sockaddr*>(&dst_), sizeof(dst_));
    return n == static_cast<ssize_t>(d.size());
  }

 private:
  int fd_ = -1;
  sockaddr_in dst_{};
  int64_t epoch_ns_ = 0;
  std::atomic<uint64_t> seq_{0};
  std::mutex send_mu_;
};

// The single path every writer uses: file (if enabled) + datagram (if publisher open).
class StateSink {
 public:
  StateSink(std::string dir, bool write_files, StatePublisher* pub)
      : dir_(std::move(dir)), files_(write_files), pub_(pub) {}

  void emit(const std::string& name, const std::string& body) {
    if (files_) {
      std::ofstream f(dir_ + "/" + name);
      if (f.is_open()) f << body;
    }
    if (pub_ && pub_->enabled()) pub_->send(name, body);
  }
  // Datagram only (used by the 1 s init-pose re-sender; the file is written once at start).
  void publish_only(const std::string& name, const std::string& body) {
    if (pub_ && pub_->enabled()) pub_->send(name, body);
  }
  const std::string& dir() const { return dir_; }
  bool files_enabled() const { return files_; }

 private:
  std::string dir_;
  bool files_;
  StatePublisher* pub_;
};

}  // namespace rt
