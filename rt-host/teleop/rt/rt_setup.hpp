// rt/rt_setup.hpp - mlockall / stack pre-fault / CPU affinity helpers and the --check-rt report.
// Scheduling policy is NOT set here: libfranka's ControlLoop makes the thread that calls
// robot.control() SCHED_FIFO/99 itself (control_tools.cpp:76).
#pragma once
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>
#include <alloca.h>
#include <dirent.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <ostream>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace rt {

// "4-15", "4,5,6", "0-1,4", "2" -> sorted unique cpu ids. Returns false on parse error.
inline bool parse_cpu_list(const std::string& s, std::vector<int>* out, std::string* err) {
  out->clear();
  if (s.empty()) {
    if (err) *err = "empty cpu list";
    return false;
  }
  size_t pos = 0;
  while (pos <= s.size()) {
    size_t comma = s.find(',', pos);
    std::string tok = s.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
    if (tok.empty()) {
      if (err) *err = "empty token in cpu list '" + s + "'";
      return false;
    }
    int lo = 0, hi = 0;
    size_t dash = tok.find('-');
    try {
      if (dash == std::string::npos) {
        lo = hi = std::stoi(tok);
      } else {
        lo = std::stoi(tok.substr(0, dash));
        hi = std::stoi(tok.substr(dash + 1));
      }
    } catch (...) {
      if (err) *err = "bad cpu token '" + tok + "'";
      return false;
    }
    if (lo < 0 || hi < lo || hi >= CPU_SETSIZE) {
      if (err) *err = "bad cpu range '" + tok + "'";
      return false;
    }
    for (int c = lo; c <= hi; ++c) {
      bool dup = false;
      for (int e : *out) dup = dup || (e == c);
      if (!dup) out->push_back(c);
    }
    if (comma == std::string::npos) break;
    pos = comma + 1;
  }
  for (size_t i = 1; i < out->size(); ++i)  // insertion sort (tiny lists)
    for (size_t j = i; j > 0 && (*out)[j - 1] > (*out)[j]; --j) std::swap((*out)[j - 1], (*out)[j]);
  return true;
}

// {4,5,6,7,10} -> "4-7,10"
inline std::string cpu_list_to_string(const std::vector<int>& cpus) {
  std::ostringstream os;
  size_t i = 0;
  while (i < cpus.size()) {
    size_t j = i;
    while (j + 1 < cpus.size() && cpus[j + 1] == cpus[j] + 1) ++j;
    if (i > 0) os << ',';
    if (j == i) os << cpus[i];
    else os << cpus[i] << '-' << cpus[j];
    i = j + 1;
  }
  return os.str();
}

// Locks memory ONLY when RLIMIT_MEMLOCK is unlimited: mlockall(MCL_FUTURE) obliges the kernel
// to lock every future page the process acquires (including stack growth and thread stacks),
// and on a finite limit a later allocation that would exceed it kills the process with SIGSEGV
// instead of a graceful failure (see mlockall(2) NOTES). So under a finite limit we skip the
// call entirely and warn instead of risking that crash.
inline bool lock_all_memory(std::string* err) {
  rlimit rl{};
  if (getrlimit(RLIMIT_MEMLOCK, &rl) == 0 && rl.rlim_cur != RLIM_INFINITY) {
    if (err) *err = "skipped mlockall: RLIMIT_MEMLOCK is " + std::to_string(rl.rlim_cur / 1024) +
                    " kB, need unlimited (realtime limits not configured)";
    return false;   // never arm MCL_FUTURE under a finite limit
  }
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    if (err) *err = std::string("mlockall failed: ") + strerror(errno);
    return false;
  }
  return true;
}

// Touch `bytes` of stack below the current frame (capped 1 MB under RLIMIT_STACK). Returns
// the number of bytes actually touched. Call AFTER mlockall so the pages stay resident.
__attribute__((noinline)) inline size_t prefault_stack(size_t bytes) {
  rlimit rl{};
  if (getrlimit(RLIMIT_STACK, &rl) == 0 && rl.rlim_cur != RLIM_INFINITY) {
    const size_t headroom = 1u << 20;
    size_t avail = rl.rlim_cur > headroom ? static_cast<size_t>(rl.rlim_cur) - headroom : 0;
    if (bytes > avail) bytes = avail;
  }
  if (bytes == 0) return 0;
  volatile unsigned char* p = static_cast<volatile unsigned char*>(alloca(bytes));
  const long page = sysconf(_SC_PAGESIZE) > 0 ? sysconf(_SC_PAGESIZE) : 4096;
  for (size_t i = 0; i < bytes; i += static_cast<size_t>(page)) p[i] = 0;
  p[bytes - 1] = 0;
  return bytes;
}

// Affinity of the CALLING thread (inherited by threads it creates afterwards).
inline bool set_self_affinity(const std::vector<int>& cpus, std::string* err) {
  cpu_set_t set;
  CPU_ZERO(&set);
  for (int c : cpus) CPU_SET(c, &set);
  if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
    if (err) *err = "setaffinity(" + cpu_list_to_string(cpus) + ") failed: " + strerror(errno);
    return false;
  }
  return true;
}

inline bool pin_self(int cpu, std::string* err) { return set_self_affinity({cpu}, err); }

inline std::vector<int> self_affinity() {
  cpu_set_t set;
  CPU_ZERO(&set);
  std::vector<int> out;
  if (pthread_getaffinity_np(pthread_self(), sizeof(set), &set) == 0)
    for (int c = 0; c < CPU_SETSIZE; ++c)
      if (CPU_ISSET(c, &set)) out.push_back(c);
  return out;
}

inline void set_thread_name(const char* name) { pthread_setname_np(pthread_self(), name); }

struct RtSetupResult {
  bool mlock_ok = false;
  std::string mlock_err;
  size_t prefaulted_bytes = 0;
  bool aux_requested = false;
  bool aux_ok = false;
  std::string aux_err;
  std::vector<int> aux_cpus;
};

// Everything that must happen at the start of main(), BEFORE helper threads are spawned.
inline RtSetupResult rt_setup_pre_threads(const std::vector<int>& aux_cpus, size_t prefault_bytes = 8u << 20) {
  RtSetupResult r;
  r.mlock_ok = lock_all_memory(&r.mlock_err);
  r.prefaulted_bytes = prefault_stack(prefault_bytes);
  if (!aux_cpus.empty()) {
    r.aux_requested = true;
    r.aux_cpus = aux_cpus;
    r.aux_ok = set_self_affinity(aux_cpus, &r.aux_err);
  }
  return r;
}

// One line per thread: tid name policy rt_priority cpus_allowed_list
inline std::string thread_table() {
  std::ostringstream os;
  DIR* d = opendir("/proc/self/task");
  if (!d) return "  (cannot read /proc/self/task)\n";
  std::vector<std::string> tids;
  while (dirent* e = readdir(d))
    if (e->d_name[0] != '.') tids.emplace_back(e->d_name);
  closedir(d);
  for (const auto& tid : tids) {
    std::string base = "/proc/self/task/" + tid + "/";
    std::ifstream st(base + "stat");
    std::string line;
    std::getline(st, line);
    std::string name = "?", policy = "?", rtprio = "?", cpus = "?";
    size_t rp = line.rfind(')');
    if (rp != std::string::npos) {
      size_t lp = line.find('(');
      name = line.substr(lp + 1, rp - lp - 1);
      std::istringstream rest(line.substr(rp + 2));
      std::vector<std::string> f;
      std::string tok;
      while (rest >> tok) f.push_back(tok);
      // f[0] = field 3 (state). rt_priority = field 40, policy = field 41 -> f[37], f[38]
      if (f.size() > 38) {
        rtprio = f[37];
        int pol = std::atoi(f[38].c_str());
        policy = pol == 0 ? "SCHED_OTHER" : pol == 1 ? "SCHED_FIFO" : pol == 2 ? "SCHED_RR" : ("policy" + f[38]);
      }
    }
    std::ifstream status(base + "status");
    while (std::getline(status, line))
      if (line.rfind("Cpus_allowed_list:", 0) == 0) {
        cpus = line.substr(18);
        size_t k = cpus.find_first_not_of(" \t");
        cpus = k == std::string::npos ? "" : cpus.substr(k);
      }
    os << "  tid=" << tid << " name=" << name << " policy=" << policy
       << " rtprio=" << rtprio << " cpus=" << cpus << '\n';
  }
  return os.str();
}

inline std::string vm_locked_line() {
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line))
    if (line.rfind("VmLck:", 0) == 0) return line;
  return "VmLck: ?";
}

// --check-rt: apply rt setup, spawn two probe helper threads (inherit the aux mask), pin self
// to rt_cpu, probe SCHED_FIFO/99 (what libfranka will do) and restore, print the report.
inline int run_check_rt(std::ostream& os, const std::vector<int>& aux_cpus, int rt_cpu) {
  RtSetupResult r = rt_setup_pre_threads(aux_cpus);
  os << "check-rt\n";
  os << "  mlockall: " << (r.mlock_ok ? "ok" : ("FAILED (" + r.mlock_err + ")")) << "\n";
  os << "  " << vm_locked_line() << "\n";
  os << "  stack_prefault_bytes: " << r.prefaulted_bytes << "\n";
  os << "  aux_cpus: " << (r.aux_requested ? cpu_list_to_string(r.aux_cpus) : std::string("(not set)"))
     << (r.aux_requested ? (r.aux_ok ? " ok" : " FAILED (" + r.aux_err + ")") : "") << "\n";
  std::atomic<bool> go{true};
  std::thread probe1([&] { set_thread_name("probe_helper1"); while (go.load()) usleep(10000); });
  std::thread probe2([&] { set_thread_name("probe_helper2"); while (go.load()) usleep(10000); });
  usleep(20000);  // let the probes set their names before the table is read
  std::string perr;
  bool pin_ok = true;
  if (rt_cpu >= 0) pin_ok = pin_self(rt_cpu, &perr);
  os << "  rt_cpu: " << (rt_cpu >= 0 ? std::to_string(rt_cpu) + (pin_ok ? " ok" : " FAILED (" + perr + ")")
                                    : std::string("(not set)")) << "\n";
  os << "  main_affinity: " << cpu_list_to_string(self_affinity()) << "\n";
  sched_param sp{};
  sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
  int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  os << "  sched_fifo_max_probe: " << (rc == 0 ? "ok" : std::string("FAILED (") + strerror(rc) + ")") << "\n";
  os << "threads (while FIFO probe active):\n" << thread_table();
  if (rc == 0) {
    sched_param sp0{};
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp0);
  }
  go.store(false);
  probe1.join();
  probe2.join();
  os << "check-rt done\n";
  return 0;
}

}  // namespace rt
