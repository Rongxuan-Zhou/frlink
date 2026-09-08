# franka-rt: replica of rog's libfranka 0.17.0 userspace
#   Ubuntu 24.04 + ros-jazzy-pinocchio 3.9.0 + libpoco-dev 1.11.0 + libfmt-dev 9.1.0 + libeigen3-dev 3.4 + gcc 13 + cmake 3.28 + git
# The source tree (~/franka on the host) is bind-mounted at /franka at run time (see bin/franka-run);
# libfranka + teleop binaries are built by docker/build-inside.sh into /franka/local and /franka/teleop.
FROM ubuntu:24.04
ARG DEBIAN_FRONTEND=noninteractive
# ROS 2 Jazzy SNAPSHOT, not the live repo: the live repo ships pinocchio 4.0.0 (2026-06-04);
# 2026-04-13 is the snapshot carrying rog's exact ros-jazzy-pinocchio 3.9.0-1noble.20260225.233616.
ARG ROS_SNAPSHOT=2026-04-13
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl gnupg \
 && rm -rf /var/lib/apt/lists/*
# NOTE (deviation from the byte-for-byte brief, see task-2-report.md "concerns"): the live ros.key
# (Open Robotics key, fingerprint C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654) does NOT cover
# snapshots.ros.org, which is signed separately by the "ROS Snapshot builder" key
# (fingerprint 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA). Without it, apt-get update against the
# 2026-04-13 snapshot fails with "NO_PUBKEY AD19BAB3CBF125EA / repository is not signed" before any
# package (incl. pinocchio) is ever installed -- this is orthogonal to package version selection.
# Fetched once on the host via `gpg --keyserver keyserver.ubuntu.com --recv-keys AD19BAB3CBF125EA`
# and exported to this file, following the brief's "download failure -> host curl once, then COPY" pattern.
COPY ros-snapshot-archive-keyring.gpg /tmp/ros-snapshot-archive-keyring.gpg
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && cat /tmp/ros-snapshot-archive-keyring.gpg >> /usr/share/keyrings/ros-archive-keyring.gpg \
 && rm /tmp/ros-snapshot-archive-keyring.gpg \
 && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://snapshots.ros.org/jazzy/${ROS_SNAPSHOT}/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2-snapshot.list
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git pkg-config \
      libpoco-dev libfmt-dev libeigen3-dev \
      ros-jazzy-pinocchio \
      iproute2 iputils-ping util-linux procps binutils \
 && rm -rf /var/lib/apt/lists/*
# Fail the build unless the userspace matches rog; record the manifest.
RUN set -e; v() { dpkg-query -W -f='${Version}' "$1" 2>/dev/null || echo absent; }; \
    [ "$(v ros-jazzy-pinocchio)" = "3.9.0-1noble.20260225.233616" ]; \
    [ "$(v libpoco-dev)" = "1.11.0-4.1build2" ]; \
    [ "$(v libfmt-dev)" = "9.1.0+ds1-2" ]; \
    case "$(v libeigen3-dev)" in 3.4.0-*) ;; *) exit 1;; esac; \
    case "$(v cmake)" in 3.28.*) ;; *) exit 1;; esac; \
    case "$(v g++)" in 4:13.*) ;; *) exit 1;; esac; \
    mkdir -p /opt/franka-rt; \
    { echo "base=ubuntu:24.04 ros_snapshot=${ROS_SNAPSHOT} built=$(date -u +%FT%TZ)"; \
      for p in ros-jazzy-pinocchio ros-jazzy-hpp-fcl ros-jazzy-coal ros-jazzy-eigenpy ros-jazzy-urdfdom libpoco-dev libfmt-dev libeigen3-dev cmake g++ gcc-13 libstdc++6 libc6; do echo "$p=$(v $p)"; done; \
    } > /opt/franka-rt/IMAGE_MANIFEST.txt; cat /opt/franka-rt/IMAGE_MANIFEST.txt
LABEL org.franka.libfranka="0.17.0" org.franka.pinocchio="3.9.0" org.franka.ros_snapshot="2026-04-13"
WORKDIR /franka
CMD ["/bin/bash"]
