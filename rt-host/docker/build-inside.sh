#!/bin/bash
# build-inside.sh — runs INSIDE the franka-rt container (as the host user via FRANKA_RUN_AS_USER=1):
#   1) libfranka 0.17.0 -> /franka/local, configured exactly like rog + install RPATH so no LD_LIBRARY_PATH/ROS sourcing is needed
#   2) the 7 teleop targets + a kIgnore-patched communication_test
set -euo pipefail
LIBFRANKA_COMMIT=4448c390ea7c6af3966bcfae6308d41cee4eb7ac
COMMON_COMMIT=cd38d0ec300b7e6864407d85d1e88e2fba31ccd5
PREFIX=/franka/local
SRC=/franka/libfranka
EX=$SRC/examples
RPATH=/franka/local/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu
JOBS=${JOBS:-12}     # ~4 GB per cc1plus peak; 62 GB host
[ -d /franka/teleop ] || { echo "run via: FRANKA_RUN_AS_USER=1 franka-run /franka/docker/build-inside.sh" >&2; exit 1; }
[ "$(git -c safe.directory='*' -C $SRC rev-parse HEAD)" = "$LIBFRANKA_COMMIT" ] || { echo "libfranka HEAD != 0.17.0 ($LIBFRANKA_COMMIT)" >&2; exit 1; }
[ "$(git -c safe.directory='*' -C $SRC/common rev-parse HEAD)" = "$COMMON_COMMIT" ] || { echo "libfranka-common submodule mismatch" >&2; exit 1; }
if [ -f /opt/ros/jazzy/setup.bash ]; then set +u; source /opt/ros/jazzy/setup.bash; set -u; fi
echo "== libfranka: cmake configure/build/install (-j$JOBS)"
mkdir -p $SRC/build && cd $SRC/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=ON \
  -DCMAKE_PREFIX_PATH="/usr;/opt/ros/jazzy" -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt \
  -DCMAKE_INSTALL_RPATH="$RPATH"
cmake --build . -j"$JOBS"
cmake --install .
echo "== teleop binaries"
cd /franka/teleop
CXX="g++ -O2 -std=c++17 -pthread -I$PREFIX/include -I$EX -I/usr/include/eigen3"
LD="-L$PREFIX/lib -lfranka -Wl,-rpath,$RPATH"
for t in cartesian_pose_servo cartesian_pose_servo_pusht goto_home goto_pose_pusht pusht_pose_servo; do
  echo "  $t"; $CXX $t.cpp $EX/examples_common.cpp $LD -o $t
done
echo "  gripper_cmd"; $CXX gripper_cmd.cpp $LD -o gripper_cmd
echo "  fk_probe";    $CXX fk_probe.cpp $LD -o fk_probe
echo "  communication_test_kignore (upstream line 37 uses kEnforce, which only 'passes' via an uninitialized bool in hasRealtimeKernel())"
sed 's/franka::Robot robot(argv\[1\]);/franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);/' $EX/communication_test.cpp > communication_test_kignore.cpp
[ "$(grep -c 'RealtimeConfig::kIgnore' communication_test_kignore.cpp)" = 1 ] || { echo "patch did not apply" >&2; exit 1; }
$CXX communication_test_kignore.cpp $EX/examples_common.cpp $LD -o communication_test_kignore
# --- PART B: RT unit tests + robot-free network stub (no libfranka/Eigen needed) ---
cd /franka/teleop
$CXX tests/test_rt_units.cpp -o tests/test_rt_units
$CXX tests/servo_net_stub.cpp -o tests/servo_net_stub
echo "== running rt unit tests"
tests/test_rt_units          # exits non-zero on any failing check -> build fails (set -e)
echo "built: tests/test_rt_units tests/servo_net_stub"
echo "== verify"
echo "LD_LIBRARY_PATH=[${LD_LIBRARY_PATH:-}]"
ldd $PREFIX/lib/libfranka.so | grep -E 'pinocchio|not found'
! ldd $PREFIX/lib/libfranka.so | grep -q 'not found'
readelf -d $PREFIX/lib/libfranka.so | grep -E 'RUNPATH|RPATH'
ls -l /franka/teleop/cartesian_pose_servo_pusht /franka/local/bin/communication_test /franka/local/bin/echo_robot_state
echo "BUILD OK"
