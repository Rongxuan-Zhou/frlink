#!/usr/bin/env bash
# bench_net.sh - no-robot bench for the servo network path using PART B's servo_net_stub.
# Usage: tests/bench_net.sh [a b c d e f]   (default: all)   env: STUB_MODE=docker|native  IMAGE=franka-rt:0.17.0-jazzy
set -uo pipefail
ROOT=/home/rongxuan_zhou/franka; T=$ROOT/tests
STUB_MODE=${STUB_MODE:-docker}; IMAGE=${IMAGE:-franka-rt:0.17.0-jazzy}
STUB_BIN=/franka/teleop/tests/servo_net_stub; STUB_NATIVE=$ROOT/teleop/tests/servo_net_stub
MIRROR=/tmp/franka_mirror; STATE_DIR=/tmp/franka_stub_state; LINK_IP=10.10.0.2; ROG_LINK_IP=10.10.0.1
DAY=$(date +%F); RUN=$T/results/$DAY/bench_net_$(date +%H%M%S); mkdir -p "$RUN"
DOCKER=docker; docker info >/dev/null 2>&1 || DOCKER="sudo docker"
FAILS=0; declare -A VERDICT

log() { echo "[$(date +%T)] $*" | tee -a "$RUN/bench.log"; }
linkval() { tr ' ' '\n' < "$MIRROR/franka_link.txt" 2>/dev/null | awk -F= -v k="$1" '$1==k{print $2}'; }
sample_link() { # $1=csv $2=seconds ; 10 Hz samples of key link values
  local end=$((SECONDS + $2)); echo "t,cmd_age_ms,cmd_pkts_last_s,cmd_drop_allow,cmd_drop_latch,latched_sender" > "$1"
  while [ $SECONDS -lt $end ]; do
    echo "$(date +%s.%N),$(linkval cmd_age_ms),$(linkval cmd_pkts_last_s),$(linkval cmd_drop_allow),$(linkval cmd_drop_latch),$(linkval latched_sender)" >> "$1"
    sleep 0.1; done; }
start_stub() { # $1=bind $2=allow $3=state_dst $4=logfile
  rm -rf "$MIRROR" "$STATE_DIR"; mkdir -p "$MIRROR" "$STATE_DIR"
  local args="--cmd-bind $1 --cmd-allow $2 --cmd-port 50001 --state-dst $3 --state-dir $STATE_DIR --rt-cpu 2 --aux-cpus 4-15"
  if [ "$STUB_MODE" = docker ]; then
    $DOCKER rm -f franka-stub >/dev/null 2>&1
    $DOCKER run -d --name franka-stub --privileged --network host --ulimit rtprio=99 --ulimit memlock=-1 \
      -v "$ROOT":/franka -v "$STATE_DIR":"$STATE_DIR" "$IMAGE" $STUB_BIN $args >/dev/null || { log "docker run failed"; return 1; }
    ( $DOCKER logs -f franka-stub > "$4" 2>&1 & )
  else
    "$STUB_NATIVE" $args > "$4" 2>&1 & echo $! > "$RUN/.stub_pid"
  fi
}
stop_stub() { # SIGTERM, wait <=5 s, report exit code and stop latency in $STOP_EXIT/$STOP_MS
  local t0=$(date +%s%N)
  if [ "$STUB_MODE" = docker ]; then
    $DOCKER kill -s TERM franka-stub >/dev/null 2>&1; timeout 5 $DOCKER wait franka-stub >/dev/null 2>&1
    STOP_EXIT=$($DOCKER inspect -f '{{.State.ExitCode}}' franka-stub 2>/dev/null); $DOCKER rm -f franka-stub >/dev/null 2>&1
  else
    local p=$(cat "$RUN/.stub_pid"); kill -TERM "$p" 2>/dev/null; timeout 5 tail --pid="$p" -f /dev/null; wait "$p" 2>/dev/null; STOP_EXIT=$?
  fi
  STOP_MS=$(( ($(date +%s%N) - t0) / 1000000 ))
}
start_sink() { python3 "$T/state_sink.py" --bind "$1" --port 50002 --mirror "$MIRROR" --duration "$2" --report "$3" ${4:-} > "$5" 2>&1 & SINK_PID=$!; }
wait_link() { for i in $(seq 100); do [ -s "$STATE_DIR/franka_link.txt" ] && return 0; sleep 0.1; done; log "link file never appeared"; return 1; }
# DEVIATION from brief: wait_link originally polled $MIRROR/franka_link.txt (written only by
# state_sink.py over UDP). Every case_* calls wait_link before start_sink, so $MIRROR is always
# still empty and the 10 s poll times out unconditionally (verified: case_a, the only case that
# aborts on wait_link's return code, always FAILed "no link file"; cases b-e ignored the return
# code and happened to succeed because their later linkval() reads occur after start_sink has had
# time to run). servo_net_stub writes the identical franka_link.txt body straight into
# --state-dir (bind-mounted, no sink required) within ~0.3 s of starting (measured), so wait_link
# now polls $STATE_DIR instead - a true "is the stub up" gate that does not depend on start_sink
# having run yet. This does not change what any case measures: all linkval()/consumer_check.py
# reads still come from $MIRROR (the sink-mirrored, network-exercised copy) later in each case.
verdict() { VERDICT[$1]=$2; echo "$2: $3" > "$RUN/case_$1/verdict.txt"; log "case $1 => $2 ($3)"; [ "$2" = PASS ] || [ "$2" = SKIP ] || FAILS=$((FAILS+1)); }
cleanup() { pkill -f "$T/fake_sender.py" 2>/dev/null; [ -n "${SINK_PID:-}" ] && kill "$SINK_PID" 2>/dev/null; $DOCKER rm -f franka-stub >/dev/null 2>&1; }
trap cleanup EXIT

case_a() { local D=$RUN/case_a; mkdir -p "$D"; log "case a: loopback happy path 60 s @90 Hz"
  start_stub 127.0.0.1 127.0.0.1 127.0.0.1:50002 "$D/stub.log" || { verdict a FAIL "stub start"; return; }
  wait_link || { verdict a FAIL "no link file"; stop_stub; return; }
  start_sink 0.0.0.0 62 "$D/sink.json" "--record-ee $D/ee.csv" "$D/sink.log"; sleep 1
  python3 "$T/fake_sender.py" --mode circle --duration 60 --rate 90 --src-ip 127.0.0.1 --src-port 40001 --log "$D/cmd.csv" > "$D/sender.log" 2>&1 &
  sleep 20; python3 "$T/consumer_check.py" --mirror "$MIRROR" --secs 20 --report "$D/consumer.json" > "$D/consumer.log" 2>&1; local cc=$?
  sleep 19; local pk=$(linkval cmd_pkts_last_s); local age=$(linkval cmd_age_ms); wait $SINK_PID; local se=$?
  local sent=$(grep -o 'DONE sent=[0-9]*' "$D/sender.log" | cut -d= -f2); stop_stub
  local ok=1; [ "$se" = 0 ] || ok=0; [ "$cc" = 0 ] || ok=0; [ "${sent:-0}" -ge 5290 ] && [ "${sent:-0}" -le 5510 ] || ok=0
  [ "${pk:-0}" -ge 85 ] && [ "${pk:-0}" -le 95 ] || ok=0; [ "${age:-999}" -lt 50 ] || ok=0; [ "$STOP_EXIT" = 0 ] || ok=0
  [ $ok = 1 ] && verdict a PASS "sink=$se consumer=$cc sent=$sent pkts/s=$pk age=$age stop_exit=$STOP_EXIT" || verdict a FAIL "sink=$se consumer=$cc sent=$sent pkts/s=$pk age=$age stop_exit=$STOP_EXIT"; }

case_b() { local D=$RUN/case_b; mkdir -p "$D"; log "case b: allowlist rejection (sender 127.0.0.2 vs allow 127.0.0.1)"
  start_stub 127.0.0.1 127.0.0.1 127.0.0.1:50002 "$D/stub.log" || { verdict b FAIL "stub start"; return; }; wait_link
  start_sink 0.0.0.0 12 "$D/sink.json" "" "$D/sink.log"; sleep 1
  python3 "$T/fake_sender.py" --mode constant --duration 5 --src-ip 127.0.0.2 --src-port 40003 > "$D/sender.log" 2>&1
  sleep 1; local da=$(linkval cmd_drop_allow); local ls=$(linkval latched_sender); local pk=$(linkval cmd_pkts_last_s)
  wait $SINK_PID; stop_stub
  if [ "${da:-0}" -gt 0 ] && [ "${ls:-none}" != "127.0.0.2:40003" ] && [ "${pk:-1}" = 0 ]; then verdict b PASS "cmd_drop_allow=$da latched=$ls pkts/s=$pk"
  else verdict b FAIL "cmd_drop_allow=$da latched=$ls pkts/s=$pk"; fi; }

case_c() { local D=$RUN/case_c; mkdir -p "$D"; log "case c: latch - second sender dropped, takes over 1 s after first stops"
  start_stub 127.0.0.1 127.0.0.1 127.0.0.1:50002 "$D/stub.log" || { verdict c FAIL "stub start"; return; }; wait_link
  start_sink 0.0.0.0 30 "$D/sink.json" "" "$D/sink.log"; sleep 1
  python3 "$T/fake_sender.py" --mode constant --duration 15 --src-ip 127.0.0.1 --src-port 40001 > "$D/senderA.log" 2>&1 &
  sleep 5; python3 "$T/fake_sender.py" --mode constant --duration 20 --src-ip 127.0.0.1 --src-port 40002 > "$D/senderB.log" 2>&1 &
  sample_link "$D/link_samples.csv" 22 &
  sleep 5; local l10=$(linkval latched_sender); local dl10=$(linkval cmd_drop_latch)      # t=10: A latched, B dropped
  sleep 7.5; local l17=$(linkval latched_sender); local age17=$(linkval cmd_age_ms)      # t=17.5: A stopped at 15 -> B latched by 16
  wait "$SINK_PID"; wait; stop_stub
  if [ "$l10" = "127.0.0.1:40001" ] && [ "${dl10:-0}" -gt 0 ] && [ "$l17" = "127.0.0.1:40002" ] && [ "${age17:-999}" -lt 100 ]; then
    verdict c PASS "t10 latched=$l10 drop_latch=$dl10 ; t17.5 latched=$l17 age=$age17"
  else verdict c FAIL "t10 latched=$l10 drop_latch=$dl10 ; t17.5 latched=$l17 age=$age17"; fi; }

case_d() { local D=$RUN/case_d; mkdir -p "$D"; log "case d: burst-then-silence -> cmd_age_ms > 200 during silence"
  start_stub 127.0.0.1 127.0.0.1 127.0.0.1:50002 "$D/stub.log" || { verdict d FAIL "stub start"; return; }; wait_link
  start_sink 0.0.0.0 14 "$D/sink.json" "" "$D/sink.log"; sleep 1
  sample_link "$D/link_samples.csv" 11 &
  python3 "$T/fake_sender.py" --mode burst --burst-on 3 --burst-off 2 --cycles 2 --src-ip 127.0.0.1 --src-port 40001 > "$D/sender.log" 2>&1
  wait; stop_stub
  python3 - "$D/link_samples.csv" <<'EOF' > "$D/analysis.txt"
import csv, sys
rows = [r for r in csv.DictReader(open(sys.argv[1])) if r["cmd_age_ms"] not in ("", "?")]
t0 = float(rows[0]["t"]); ages = [(float(r["t"]) - t0, float(r["cmd_age_ms"])) for r in rows]
silence1 = [a for t, a in ages if 3.3 <= t <= 4.9]; burst2 = [a for t, a in ages if 5.6 <= t <= 7.9]
print("max_age_silence1", max(silence1) if silence1 else -1, "max_age_burst2", max(burst2) if burst2 else -1)
ok = silence1 and max(silence1) > 200 and burst2 and max(burst2) < 60
print("RESULT", "PASS" if ok else "FAIL")
EOF
  cat "$D/analysis.txt"; grep -q "RESULT PASS" "$D/analysis.txt" && verdict d PASS "$(head -1 "$D/analysis.txt")" || verdict d FAIL "$(head -1 "$D/analysis.txt")"; }

case_e() { local D=$RUN/case_e; mkdir -p "$D"; log "case e: SIGTERM -> exit 0 within 1 s"
  start_stub 127.0.0.1 127.0.0.1 127.0.0.1:50002 "$D/stub.log" || { verdict e FAIL "stub start"; return; }; wait_link; sleep 2
  stop_stub; echo "exit=$STOP_EXIT stop_ms=$STOP_MS" > "$D/stop.txt"
  if [ "$STOP_EXIT" = 0 ] && [ "$STOP_MS" -le 1000 ]; then verdict e PASS "exit=$STOP_EXIT in ${STOP_MS}ms"; else verdict e FAIL "exit=$STOP_EXIT in ${STOP_MS}ms"; fi; }

case_f() { local D=$RUN/case_f; mkdir -p "$D"; log "case f: private-link IP ($LINK_IP) both ends on alienware + ping to rog"
  if ! ip -4 addr show | grep -q "inet $LINK_IP/"; then verdict f SKIP "franka-link ($LINK_IP) not configured/up"; return; fi
  start_stub $LINK_IP "$LINK_IP,$ROG_LINK_IP" $LINK_IP:50002 "$D/stub.log" || { verdict f FAIL "stub start"; return; }; wait_link
  start_sink $LINK_IP 32 "$D/sink.json" "" "$D/sink.log"; sleep 1
  python3 "$T/fake_sender.py" --mode circle --duration 30 --dst $LINK_IP:50001 --src-ip $LINK_IP --src-port 40001 > "$D/sender.log" 2>&1 &
  local pingv="SKIP"; if ping -c 2 -W 1 $ROG_LINK_IP >/dev/null 2>&1; then
    ping -c 2000 -i 0.002 $ROG_LINK_IP > "$D/ping_rog.txt" 2>&1
    local loss=$(grep -o '[0-9.]*% packet loss' "$D/ping_rog.txt" | cut -d% -f1); local mx=$(grep -o 'rtt.*' "$D/ping_rog.txt" | cut -d/ -f6)
    pingv="loss=${loss}% max=${mx}ms"; awk -v l="$loss" -v m="$mx" 'BEGIN{exit !(l==0 && m<5)}' || pingv="FAIL $pingv"; fi
  wait "$SINK_PID"; local se=$?; wait; stop_stub
  if [ "$se" = 0 ] && [ "$STOP_EXIT" = 0 ] && [[ "$pingv" != FAIL* ]]; then verdict f PASS "sink=$se ping[$pingv]"; else verdict f FAIL "sink=$se ping[$pingv]"; fi; }

CASES=${*:-a b c d e f}
log "bench_net start STUB_MODE=$STUB_MODE IMAGE=$IMAGE RUN=$RUN"
for c in $CASES; do case_$c; sleep 1; done
{ echo "# bench_net $DAY $(basename "$RUN")"; echo; echo "| case | verdict | detail |"; echo "|---|---|---|"
  for c in $CASES; do echo "| $c | ${VERDICT[$c]} | $(cut -d: -f2- "$RUN/case_$c/verdict.txt") |"; done
  echo; echo "STUB_MODE=$STUB_MODE IMAGE=$IMAGE host=$(hostname) kernel=$(uname -r) fails=$FAILS"; } > "$RUN/summary.md"
cat "$RUN/summary.md"; exit $FAILS
