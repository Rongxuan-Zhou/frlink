// rt/cli.hpp - pre-scan argv for the PART B flags and strip them, so the legacy
// `for (int i = 2; i < argc; i += 2)` loops in the servos stay untouched.
#pragma once
#include <cstdlib>
#include <string>
#include <vector>

namespace rt {

struct RtCliOptions {
  std::string cmd_bind = "127.0.0.1";
  std::string cmd_allow = "127.0.0.1";
  std::string state_dst;         // "" = no publisher
  std::string state_dir;         // resolved: --state-dir, else $FRANKA_STATE_DIR, else /tmp
  bool state_files = true;
  int rt_cpu = -1;               // -1 = don't pin
  std::string aux_cpus;          // "" = don't set
  bool help = false;
  bool check_rt = false;

  // argv with the new flags removed (argv[0] kept; `--cmd-port N` re-emitted as `--port N`).
  std::vector<std::string> rest_storage;
  std::vector<char*> rest_argv;
  int rest_argc() const { return static_cast<int>(rest_argv.size()); }
  char** rest_argv_ptr() { return rest_argv.data(); }
};

inline std::string resolve_state_dir(const std::string& cli_value) {
  if (!cli_value.empty()) return cli_value;
  const char* env = std::getenv("FRANKA_STATE_DIR");
  if (env && env[0] != '\0') return env;
  return "/tmp";
}

// Returns false (with *err set) on a missing value or a bad --state-files / --rt-cpu number.
inline bool prescan_rt_args(int argc, char** argv, RtCliOptions* o, std::string* err) {
  o->rest_storage.clear();
  o->rest_argv.clear();
  if (argc > 0) o->rest_storage.emplace_back(argv[0]);
  std::string cmd_port;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const char* flag, std::string* dst) -> bool {
      if (i + 1 >= argc) {
        if (err) *err = std::string(flag) + " needs a value";
        return false;
      }
      *dst = argv[++i];
      return true;
    };
    if (a == "--help" || a == "-h") { o->help = true; continue; }
    if (a == "--check-rt") { o->check_rt = true; continue; }
    if (a == "--cmd-bind")  { if (!need("--cmd-bind", &o->cmd_bind)) return false; continue; }
    if (a == "--cmd-allow") { if (!need("--cmd-allow", &o->cmd_allow)) return false; continue; }
    if (a == "--cmd-port")  { if (!need("--cmd-port", &cmd_port)) return false; continue; }
    if (a == "--state-dst") { if (!need("--state-dst", &o->state_dst)) return false; continue; }
    if (a == "--state-dir") { if (!need("--state-dir", &o->state_dir)) return false; continue; }
    if (a == "--aux-cpus")  { if (!need("--aux-cpus", &o->aux_cpus)) return false; continue; }
    if (a == "--state-files") {
      std::string v;
      if (!need("--state-files", &v)) return false;
      if (v != "0" && v != "1") { if (err) *err = "--state-files wants 0 or 1"; return false; }
      o->state_files = (v == "1");
      continue;
    }
    if (a == "--rt-cpu") {
      std::string v;
      if (!need("--rt-cpu", &v)) return false;
      try { o->rt_cpu = std::stoi(v); } catch (...) { if (err) *err = "--rt-cpu wants an integer"; return false; }
      continue;
    }
    o->rest_storage.push_back(a);
  }
  if (!cmd_port.empty()) {  // alias: legacy loop parses --port
    o->rest_storage.emplace_back("--port");
    o->rest_storage.push_back(cmd_port);
  }
  o->state_dir = resolve_state_dir(o->state_dir);
  for (auto& s : o->rest_storage) o->rest_argv.push_back(const_cast<char*>(s.c_str()));
  return true;
}

inline std::string rt_flags_help() {
  return
      "  --cmd-bind <ip>          UDP command bind address (default 127.0.0.1; 0.0.0.0 allowed)\n"
      "  --cmd-port <port>        alias of --port (default 50001)\n"
      "  --cmd-allow <ip[,ip..]>  accepted source IPs (default 127.0.0.1); first (ip,port) is\n"
      "                           latched, others dropped until 1000 ms of silence\n"
      "  --state-files 0|1        write franka_*.txt state files (default 1)\n"
      "  --state-dir <dir>        state file dir (default $FRANKA_STATE_DIR or /tmp)\n"
      "  --state-dst <ip:port>    also publish every state body as a FRST1 UDP datagram\n"
      "  --rt-cpu <n>             pin the control thread to cpu n (default: no pin)\n"
      "  --aux-cpus <list>        run helper threads on these cpus, e.g. 4-15 or 4,5,6\n"
      "  --check-rt               apply rt setup, print affinity/policy report, exit 0\n"
      "  --help                   this text\n";
}

}  // namespace rt
