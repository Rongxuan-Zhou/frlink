[English](INTERFACE.md) | 中文

# FR3 两主机接口契约
Frozen 2026-09-02; any change bumps FRST1→FRST2 and the cmd_allow set.（2026-09-02 冻结；任何改动都会把 FRST1→FRST2 升级，并更新 cmd_allow 集合。）
2026-09-08：任务专用变体已从这份公开副本中移除；接口本身没有变化。

## 2. 两主机接口契约（由本计划冻结；rog 侧按它来实现）

### 2.1 物理链路与地址
| 链路 | 本机接口 | 对端 | 地址 | 用途 |
|---|---|---|---|---|
| A 机器人链路（硬实时） | `enp110s0`（RTL8126，r8126） | FR3 Control | 本机 172.16.0.1/24 ↔ 机器人 172.16.0.2 | FCI UDP 1 kHz + Desk HTTPS |
| B 控制面（非实时） | `frlink0`（USB 扩展坞端口，ASIX AX88179A，MAC 00:0a:cd:48:1e:dd，驱动 ax88179_178a），直连网线到 rog 的 `eno1`，无交换机 | rog | 私有子网 10.10.0.2/24 ↔ rog 10.10.0.1 | 命令 / 状态 / ssh / chrony |
| C 互联网访问 | WiFi | 校园网 | DHCP | ssh/apt，永远不在控制路径上 |

### 2.2 命令流（rog → 本机）：UDP `10.10.0.2:50001`
- 载荷：**128 字节** = 16 × float64 小端，列主序 4x4 齐次变换 `O_T_EE`（平移量在索引 12,13,14），与现有的 `struct.pack("16d", *T.flatten(order="F"))` 完全相同。
- 其它任何长度都丢弃并计数。无序号，无校验和（为兼容而保留）。
- 只接受 `--allow-src` 白名单中的源 IP；第一个有效发送端被 latch，其它任何来源的数据包都丢弃并计数，latch 在静默 1 s 后释放。
- 语义不变：> 200 ms 没有数据包 → servo 把目标锁定到当前 `O_T_EE`（保持；力矩控制继续）。

### 2.3 状态流（本机 → rog）：UDP `10.10.0.1:50002`
每个数据报 = 一行 ASCII 头 + 原样的文件体（与 /tmp 文件的写法逐字节相同；rog 侧的 mirror 程序只是把它原封不动写到磁盘）：
```
FRST1 <seq> <epoch> <t_real_ns> <t_mono_ns> <name>\n<file body>
```
- `name` ∈ {`franka_init_pose.txt`（启动时一次，之后每 1 s 重发）, `franka_current_ee.txt`（20 Hz）, `franka_wrench.txt`（20 Hz）, `franka_joint_state.txt`（10 Hz）, `franka_link.txt`（5 Hz，新增）}。
- `seq` 在进程内单调递增（uint64）；`epoch` = servo 进程启动时的 `CLOCK_REALTIME` ns；rog 侧 mirror 看到 epoch 变化 → 立即重写 `franka_init_pose.txt`（修复 B 键重启后参考位姿陈旧的问题）。
- **文件体字节格式（逐字照抄自 rog 源码 `teleop/cartesian_pose_servo.cpp:213-248`）**：普通 `std::ofstream`，**完全不用 `setprecision`/`fixed`**，即 `defaultfloat`、6 位有效数字（`%g` 风格；在 `[1e-5,1e6)` 之外用 `e±0X` 指数）；按 `f << v[i] << (i + 1 < N ? ' ' : '\n')` 写出：单行、单空格分隔、末尾一个 `\n`、无头、无尾随空白。N = 16（init_pose / current_ee，列主序 `O_T_EE`，平移量在 12/13/14）、6（wrench = `O_F_ext_hat_K`：Fx Fy Fz Tx Ty Tz）、28（joint = q[7] dq[7] tau_J[7] tau_ext_hat_filtered[7]）。
- rog 侧消费者只通过 `f.read().split()` 数 token（16/6/28）再 `float()`；客户端状态读取器要求 EE 文件的 **mtime 在 250 ms 内前进**，deploy/auto_collect 要求 < 0.5 s；03 桥接程序启动时最多等 `franka_init_pose.txt` 5 s，且在进程内**从不重新读取它**。
- `franka_link.txt`（新增，5 Hz）：一行空格分隔的 `key=value`：`cmd_age_ms cmd_pkts_last_s cmd_drop_size cmd_drop_allow cmd_drop_latch latched_sender missed_cycles_total max_consecutive_missed freeze recovering tick_over_1p2ms_1s tick_max_us_1s reflex_count`。
- 文件写入和网络发送必须由**同一个格式化函数**产生同一个字符串（由构造保证字节一致）。

### 2.3a Servo 生产参数（照抄自 rog 工作树中的启动脚本）
- 常规遥操作：`cartesian_pose_servo 172.16.0.2`（`launch_live.sh:39`，全部默认值：port 50001，alpha 0.05，K_t 1000，K_r 80，负载 0.25 kg / COM z 0.05）。
- Home：`goto_home 172.16.0.2 --speed 0.10`（出厂就绪关节构型）。
- Servo 行为常量不变：200 ms 无数据包后保持；F/T 紧急冻结阈值 |F|>35 N / |T|>20 Nm，持续 1500 ms；力矩钳位 0.85×[87,87,87,87,12,12,12]；`franka::limitRate`；reflex 重试 ≤ 50。
- **rog 上运行的二进制来自工作树，而不是 HEAD**（工作树额外包含：wrench/joint 文件写入、`--load-com-*`、`goto_home --q/--mode`）。

### 2.4 控制动词（rog → 本机）：ssh forced-command `franka-ctl`
只允许以下动词：`status` · `preflight` · `start pose` · `stop` · `restart pose [home]`（默认 = stop → start，不移动手臂；带 `home` = stop → 回 home 序列 → start，即 rog 上 B 键今天做的事）· `goto home`（servo 运行时拒绝）· `gripper <argv...>`（转发给本机的 `gripper_cmd`，与今天的 CLI 完全相同）· `echo`（`echo_robot_state`，servo 运行时拒绝）· `comm-test --yes`（`communication_test` 的 kIgnore 变体；它会先把手臂移到出厂位姿，`--yes` 必填）。所有触碰机器人的动词共用 `flock -n /run/lock/franka-robot.lock`（例外：`gripper`，它可以与 servo 共存，使用 `/run/lock/franka-gripper.lock`）。退出码：程序的退出码原样透传；64 = 用法错误 / 被拒绝的动词；75 = 忙（锁被占或 servo 在运行）。

### 2.5 rog 侧需要的最小改动（不在本计划范围内；仅为对齐而列出）
`franka_state_mirror`（在 50002 上接收 → 原子地写四个 /tmp 文件；250 ms 无数据包后停止写入）；`gripper_cmd`/`goto_*`/`echo_robot_state` 封装脚本放在相同路径（`FRANKA_SERVO_HOST` 未设置时本地执行，否则走 ssh franka-ctl）；三个发送端读取 `FRANKA_SERVO_HOST`；客户端启动脚本变成一个编排器。
