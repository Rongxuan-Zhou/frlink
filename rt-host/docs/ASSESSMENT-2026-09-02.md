# FR3 控制栈双机分工评估（2026-09-02）

问题：能否让 Alienware m18 R2（本机）承担 libfranka 1 kHz 实时控制，rog 台式机负责脚本执行 / 策略推理 / 数据采集？或者别的组合更合适？

---

## 1. 两台机器现状

| 项目 | rog（台式机，现有全栈主机） | Alienware（本机） |
|---|---|---|
| OS / 内核 | Ubuntu 24.04, 6.17.0-23 `PREEMPT_DYNAMIC`（默认 voluntary） | Ubuntu 22.04.5, 6.8.0-124 `PREEMPT_DYNAMIC`（默认 voluntary） |
| 内核可用调优 | `preempt=full`/`isolcpus`/`nohz_full`/`irqaffinity`/`threadirqs` 均可用，HZ=1000 | 同上 |
| CPU | Ultra 9 285K：8 P-core（cpu0-7，无超线程）+ 16 E-core | i9-14900HX：8 P-core 带超线程（cpu0-15，兄弟 0-1/2-3…）+ 16 E-core（16-31） |
| GPU | RTX 5080 16 GB（nvidia 595 open） | RTX 4070 Laptop 8 GB（nvidia-dkms 580，hybrid 模式） |
| 实时权限 | `rtprio 99` + `memlock unlimited`（limits.d），用户在 `realtime` 组 | `rtprio 0`，无 realtime 组（**需 sudo 配置**） |
| 电源/空闲 | governor powersave，C3 退出延迟 1048 µs | 同上；AC 供电；GNOME **合盖=suspend** |
| 机器人网口 | `eno1` RTL8125，in-tree `r8169`，静态 172.16.0.1/24（`franka-fci`），**当前 link DOWN** | 唯一有线口 `enp110s0` RTL8126（10ec:5000），**仅 DKMS `r8126` 支持**，现为校园网上联 |
| 第二网口 | USB RTL8153 千兆（校园网） | 无（有 TB4 口和 USB-C，可外接） |
| 自动更新 | — | `unattended-upgrades` 开启 → 内核升级会触发 r8126/nvidia DKMS 重编 |
| Ubuntu Pro RT 内核 | 未 attach（RESUME.md P2 计划） | 未 attach；免费个人版可装 `linux-realtime-hwe-22.04`（6.8 RT） |
| libfranka | 0.17.0 源码编译到 `~/franka/local`（pinocchio 3.9 来自 ros-jazzy，fmt 9，Poco 1.11） | 无；22.04 上需重编（fmt 8、`ros-humble-pinocchio` 3.5、`libpoco-dev`），或用 Docker 跑 24.04 用户态 |
| franky | `.venv_franky` py3.13，franky-control 1.1.3（libfranka-0.17.0 专用 wheel） | 可装 cp310 同版本 wheel，自带 libfranka，不需系统库 |

机器人：FR3，System Image **5.7.2 锁定** → libfranka 必须在 [0.15, 0.18)，research-interface server v9。

## 2. 代码架构（rog `~/franka`，只读调研）

- **实时执行器** = C++ 力矩伺服 `teleop/cartesian_pose_servo.cpp` / `_pusht.cpp`：`robot.control(torque cb)` + 自写笛卡尔阻抗 PD（每 tick 调 `franka::Model::coriolis/zeroJacobian`，pinocchio 是运行时硬依赖），`franka::limitRate`，F/T 紧急冻结（35 N / 20 Nm，1.5 s），reflex 自动恢复（50 次），零空间关节限位。全部 `RealtimeConfig::kIgnore`。C++ 里**没有** `sched_setscheduler`/`mlockall`/绑核，实时优先级全靠 launch 脚本 `ulimit -r 99; chrt -f 99 taskset -c 0-3`。
- **命令入口**：UDP **绑 127.0.0.1:50001**（`INADDR_LOOPBACK`，只有 `--port` 无 `--bind`），一包 = 128 字节 = 16 个 double 列优先 4x4 目标位姿；无序号/时间戳/校验；>200 ms 无包 → target 锁到当前 EE（保持，不停控）。发送方：WebXR bridge ~90 Hz（`--udp-host` 可改）、`deploy_pusht_lewm.py` 10 Hz（**硬编码** 127.0.0.1）、`auto_collect/io_utils.py` 10 Hz（硬编码）。单发送方靠纪律。
- **状态回传**：**没有网络通道**。伺服写 `/tmp/franka_init_pose.txt`（启动一次，bridge 等 5 s 否则退出）、`/tmp/franka_current_ee.txt` 20 Hz、`_wrench.txt` 20 Hz、`_joint_state.txt` 10 Hz；六个 Python 消费者靠本地文件 + mtime 判活（250/500 ms 阈值）。
- **夹爪**：Python bridge 直接 spawn `teleop/gripper_cmd`（libfranka TCP 到 172.16.0.2，硬编码 `LD_LIBRARY_PATH`）；Quest B 键 → `launch_pusht.sh restart` 本地 pkill/relaunch。
- **遥操作输入链**：Quest 3 → Tailscale Funnel（绑 rog 节点身份，因校园 WiFi 客户端隔离）→ uvicorn:4443 → WebSocket ~90 Hz → Python 回调（镜像/缩放/bbox/径向裁剪/步长钳制）→ UDP。
- **相机**：2×D455 + 1×D405 在 rog USB；录制用 RealSense 硬件时戳 + `time.time()`，EE 状态从 /tmp 文件 20 Hz 轮询、同机墙钟对齐。
- **策略**：`deploy_pusht_lewm.py` 在 CUDA 上跑 LeWM/PRISM-MPPI（0.3–0.7 s/次 5 步），与 UDP 发送同线程；规划期间伺服靠 200 ms 保持。需要 5080 级 GPU。
- **franky**：只在 `franky_tools/` 11 个独立工具里用（进程内 libfranka，慢速点到点，需伺服关闭），**不是拆分阻塞点**。ROS 2 未被任何控制路径使用。
- 伺服源码有未提交改动（+63/+115 行：wrench/joint 文件写入、零空间姿态项、4 步回 home），UDP 协议字节级未变；`technical.md` 的 "UDP 6000" 和 NaN 回 home 包是过时文档。

## 3. 实测：1 kHz 唤醒延迟（`~/rt_probe/jit`，clock_nanosleep 绝对定时，mlockall）

| 机器 / 条件 | p50 | p99 | p99.9 | max | >1 ms 次数 |
|---|---|---|---|---|---|
| rog FIFO80 P-core cpu1（30 s） | 12 µs | 194 | 247 | **5903** | 6 |
| rog FIFO80 P-core cpu3（20 s） | 8 | 19 | 33 | **6138** | 6 |
| rog FIFO80 P-core cpu7（20 s） | 149 | 260 | 285 | **5679** | 5 |
| rog FIFO80 E-core cpu12（30 s） | 3 | 4 | 95 | 215 | 0 |
| rog FIFO80 E-core cpu20（20 s，两次） | 3 | 6 | 149 | 225 / **5893** | 0 / 6 |
| Alienware OTHER（无 rtprio）P-core cpu2 | 56 | 236 | 2660 | 2664 | 134 |
| **Alienware FIFO80 P-core cpu2（sudo，30 s）** | 6 | 9 | 87 | **416** | **0** |
| Alienware FIFO80 E-core cpu20 | 8 | 16 | 117 | 283 | 0 |
| Alienware FIFO80 不绑核 | 8 | 10 | 109 | 747 | 0 |
| Alienware FIFO80 cpu2 + `cpu_dma_latency=0` | 1 | 4 | 18 | 533 | 0 |
| Alienware FIFO80 cpu2，**负载**（20 忙循环 + dd，load 8，封装 101 °C） | 1 | 15 | 197 | 575 | 0 |
| Alienware FIFO80 E-core cpu20，负载 | 4 | 14 | 31 | 503 | 0 |

- Alienware 全程 SMI 计数为 0（MSR 0x34）。hwlat 追踪 40 s（紧接负载测试、约 100 °C）：4 次 >50 µs 硬件停顿，最大 671 µs，非 SMI，疑似热/频率事件。
- rog 的 ≈6 ms 停顿**间歇出现在任意核**（每 3~5 s 一次），非 P-core 专属；无 root 无法跑 hwlat / 读 SMI 计数，原因未定。按 Franka 文档"连续 20 包丢失才停机"，6 ms ≈ 6 包，**不致命但会拉低成功率**；5 月记录的 `communication_test` Avg 1.00 与之并存的解释只能是当时没有此停顿、或 10 s 窗口未采到。
- 结论：**拿到 SCHED_FIFO 后 Alienware 比今天的 rog 更干净**；之前 OTHER 模式的坏数据全是缺实时优先级被 Claude Code/Docker 等抢占所致。

## 4. 实测：两机网络

| 路径 | 结果 |
|---|---|
| 校园有线，UDP 回显 @1 kHz，20000 包 | 0 丢包；RTT min 127 µs，p50 1045，p99 1738，max 2477 µs |
| 校园有线，UDP @100 Hz | p50 1612，max 3300 µs |
| ICMP 2000 包 | avg 1.45，max 2.19 ms |
| WiFi（wlp109s0f0）@100 Hz | p50 3.1 ms，**p99 40 ms，max 317 ms** → 不可用 |
| Tailscale | 2~51 ms 抖动 → 不可用 |
| ZeroTier | 175 ms → 不可用 |

两机时钟：均 systemd-timesyncd 对 ntp.ubuntu.com，互差 ≈0.3 ms（±0.6 ms），远小于 20 Hz 状态采样间隔。

## 5. 方案对比

（待 Workflow 2 结果补充）

## 6. 建议与验收

（待补充）
