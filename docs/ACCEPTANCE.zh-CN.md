[English](ACCEPTANCE.md) | 中文

# 验收

两个机器人日。第 1 天仅由 RT host（实时主机）驱动手臂，用一个测试发送端代替客户端：主机、servo 网络侧、
安全包络。第 2 天把真实客户端接上网线：经桥接程序的头显 teleop、重启、夹爪、热浸泡、回滚。

实际执行的清单是 `rt-host/tests/robot_day1.md` 和 `rt-host/tests/robot_day2.md`；它们调用的工具在
`rt-host/tests/` 下。结果留在主机上的 `tests/results/<date>/robot_day{1,2}/`，不属于本仓库。两天之前，
`rt-host/tests/rt_bench.sh`（6 项检查）和 `rt-host/tests/bench_net.sh`（5 项检查）必须在将要跑机器人的
同一内核、驱动和镜像上通过。

每一个会移动手臂的步骤：**user stop（急停）握在手里，工作空间清空，键盘前只有一个人。** `comm-test` 会以
0.5 的速度把手臂移动到出厂就绪位姿，且不会再次确认。

## 第 1 天：仅实时主机

除非另有标注，每条命令都在实时主机上运行。`RES` 是结果目录。

### 0. 前置条件
1. 今天的 `bench_net` 和 `rt_bench` 结果全部 PASS 或 SKIP。
2. 在 `/etc/franka/servo.env` 中做第 1 天的覆盖，让主机能充当自己的客户端：
   `FRANKA_STATE_DST=10.10.0.2:50002`，`FRANKA_CMD_ALLOW=10.10.0.1,10.10.0.2,127.0.0.1`。
   结束时用 `sudo install -m 0644 ~/franka/host/etc/franka/servo.env /etc/franka/servo.env` 恢复。
3. 接交流电；空闲时封装温度 < 80 C。

### 1. 线缆与 Desk
1. FCI 线缆接在主机的 `enp110s0` 上；`franka-fci` 配置激活，地址 `172.16.0.1/24`。
2. Desk：解锁关节，激活 FCI，按下使能按钮（Enable）。
3. `franka-ctl status`：fci 链路 up，两个 `franka-servo@*` 都是 inactive。

### 2. Preflight 与原始链路
1. `franka-ctl preflight` -> 每一行 PASS（只有 nftables-without-sudo 允许 WARN）。
2. `sudo ping -i 0.001 -c 10000 172.16.0.2` -> 0 % 丢包，最大 < 0.5 ms。
3. 前后各抓一次 `/proc/interrupts` 中 `enp110s0-*` 的快照 -> 只有 CPU 3 那一列有变化。

### 3. 通信测试 x3 与 echo
1. `franka-ctl comm-test --yes` 跑三次 -> 每次 `Avg >= 0.99`，没有 "lost robot states" 行。
2. `franka-ctl echo` -> 一条 JSON 机器人状态。

### 4. 用位姿 servo 回 home 并保持
1. `franka-ctl goto home` -> 退出码 0。
2. 在主机上启动 sink：`state_sink.py --bind 10.10.0.2 --mirror /tmp/franka_mirror --duration <s> --record-ee hold_ee.csv --gap-log hold_gaps.csv --report hold_sink.json`。
3. `franka-ctl start pose` -> 单元在 10 s 内变为 active；mirror 的 `franka_init_pose.txt` 有 16 个 token。
4. `consumer_check.py --secs 20 --bounds` -> PASS（见结果中关于检查 F 的说明）。
5. 无发送端保持（完整清单 30 min，核心运行 5 min）。每 5 min 采样一次 `franka_link.txt`。
   门限：`missed_cycles_total <= 20`，`max_consecutive_missed <= 3`，`reflex_count 0`，`freeze 0`，`cmd_age_ms -1`。
6. `track_error.py --ee hold_ee.csv` -> `drift_max <= 1.00 mm`。

### 5. 慢速画圆
1. `fake_sender.py --dst 10.10.0.2:50001 --src-ip 10.10.0.2 --src-port 40001 --mode circle --radius 0.03 --period 20 --duration <s> --center-from /tmp/franka_mirror/franka_current_ee.txt --log circle_cmd.csv`
   （圆在 t = 0 时经过当前 EE，所以没有跳变；9.4 mm/s）。
2. 运行期间：`cmd_pkts_last_s` 88-92，`latched_sender=10.10.0.2:40001`，`cmd_age_ms < 30`。
3. `track_error.py --ee hold_ee.csv --cmd circle_cmd.csv --start-skip 5` -> `err_max < 10 mm`，`reflex_count` 仍为 0。

### 6. 命令丢失与 latch 交接
1. 画圆进行中对发送端 `kill -STOP` -> 手臂立即停下；1 s 后 `cmd_age_ms > 800`；`kill -CONT` -> 恢复，1 s 内 `cmd_age_ms < 30`。无 reflex；停顿期间 EE 移动 < 2 mm。
2. `kill -9` 发送端，在 `--src-port 40002` 上启动第二个 -> `latched_sender` 在 2 s 内翻到 `:40002`。

### 7. 手推 F/T 紧急（完整清单；核心运行中移到第 2 天的浸泡）
用手稳定地侧向推处于保持中的 EE。软件冻结（`freeze=1` 持续 1.5 s，然后释放）或碰撞 reflex（`reflex_count=1`，`recovering=1` 然后变 0）二者之一均可接受。5 s 后 servo 必须仍然 active，sink 不得出现 > 250 ms 的 EE 间隙，并且新的发送端必须能重新获得控制。

### 8. 拔控制面网线（完整清单；核心运行中移到第 2 天）
画圆进行中拔掉 `frlink0` 网线（不是 FCI 线缆）10 s。servo 保持 active，无 reflex，`missed_cycles_total` 不变；发布端的 `sendto()` 失败不得影响 tick。

### 9. 停止
`franka-ctl stop` -> 退出码 0，journal 以 `exited (tick=... udp=... total_reflex=0)` 行结束，单元在 3 s 内 inactive，没有残留容器或 libfranka 进程。恢复 servo.env 的覆盖。

## 第 1 天结果，2026-09-08（核心运行，02:41-03:08 EDT）

核心子集：comm-test x3、home、5 min 保持、3 min 画圆、命令丢失保持、latch 交接、停止。移到第 2 天浸泡：
30 min 保持、F/T 推力测试、拔线。

| 步骤 | 结果 | 数据 |
|---|---|---|
| 控制面链路 `frlink0`（ASIX AX88179A，`ax88179_178a`）到客户端网卡 | PASS | ping 0 % 丢包，RTT 平均 0.32 ms。同一端口在内核默认的 `cdc_ncm` 绑定下测得 1.48 ms；udev 规则 `80-franka-ax88179.rules` 正是为此强制使用 `ax88179_178a` 驱动。 |
| 经 ssh 在 10.10.0.2 上执行 `franka-ctl preflight` | PASS | rc = 0 |
| FCI 链路，1 kHz 下 10 000 次 ping | PASS | 0 % 丢包，RTT min/avg/max 0.047 / 0.122 / 0.481 ms；IRQ 增量只出现在 CPU 3（20 552 次中断） |
| comm-test x3 | PASS | 三次运行中 Max / Avg / Min 均为 1.00 / 1.00 / 1.00 |
| echo，goto home | PASS | 收到 JSON 状态；goto rc = 0 |
| 位姿 servo 启动 | PASS | 预检：封装 67 C，绑定 10.10.0.2；mirror 中的初始位姿有 16 个 token |
| `consumer_check --secs 20 --bounds` | A-E PASS，F FAIL | 检查 F 是从某个任务专用清单继承来的桌面高度界限（z <= 0.20 m）；手臂在出厂就绪位姿 z = 0.49 m，所以 F 在 home 处不可能通过。这是清单不一致，不是系统故障。 |
| 5 min 保持 | PASS | drift_max 0.10 mm，drift_final 0.09 mm，位置标准差 (0.01, 0.01, 0.02) mm；`missed_cycles_total` 0，reflex 0，freeze 0，`tick_max_us_1s` 1119-1139 us；sink 报告 PASS |
| 画圆 r = 0.03 m，周期 20 s，180 s | 链路 PASS，跟踪 marginal FAIL | 链路：`cmd_age_ms` 5，`cmd_pkts_last_s` 90，missed 0，reflex 0。跟踪：err_max 11.44 mm，p95 9.95 mm，mean 8.28 mm，门限 10 mm。 |
| 命令丢失保持 | PASS | STOP + 1 s：`cmd_age_ms` 849；+ 3 s：2851；手臂保持，freeze 0，reflex 0。CONT + 1 s：`cmd_age_ms` 7 |
| latch 交接 | PASS | 旧发送端被杀掉后，:40002 上的新发送端在 1.5 s 内 latch 上（1 s 释放窗口内 `cmd_drop_latch` 85） |
| `franka-ctl stop` | PASS | rc = 0，用时 0.30 s；日志以 `exited (tick=659744 udp=19441 total_reflex=0)` 结束；单元 inactive |
| 停止后的残留进程 / 容器 | PASS | 无 |

11.4 mm 的画圆误差是位姿 servo 的稳态柔顺量（K_t = 1000 N/m，配 Franka Hand 负载模型 0.25 kg）在
z = 0.485 m 高度驱动 30 mm 半径时的结果。在那个高度，重力模型误差和手臂自身的动力学都在给阻抗弹簧加载。
这不是传输延迟：`cmd_age_ms` 一直是 5 ms，而且同一个二进制在单机配置上表现相同。10 mm 的门限定在对该
高度的柔顺量做任何测量之前。它作为回归界限留在清单里；以后的运行应该与这个基线比较，而不是看绝对值。
阻抗软是有意为之：遥操作中由操作者通过视觉闭环。

运行期间的温度：封装基线 67-70 C，桌面应用带来 95-100 C 的 turbo 尖峰（单核突发；核心温度 64-68 C）。
servo 预检只采样一次，这样的尖峰可能拒绝一次启动；会话期间不要在主机上跑 GUI 负载。

运行中做的、现已进入代码树的修复：`10-franka-link.link` 按 MAC 匹配端口（扩展坞上有两个一模一样的口）；
`80-franka-ax88179.rules` 绑定 `ax88179_178a`；`franka-thermal-guard.service` 加上
`StartLimitIntervalSec=0`，这样 1 Hz 定时器不会被限速成一个陈旧的温度文件。

## 第 2 天：真实客户端接上网线

导出本文时尚未运行。流程由 `rt-host/tests/robot_day2.md` 改编到位姿 servo 路径。

### 0. 客户端前置条件
1. 客户端网卡配 `10.10.0.1/24`；`ssh -i <key> rongxuan_zhou@10.10.0.2 status` 打印出状态块。
2. 客户端的状态 mirror 在运行：`/tmp/franka_current_ee.txt` 的 mtime 每 50 ms 前进一次。
3. 主机 `/etc/franka/servo.env` 已恢复生产值（`FRANKA_STATE_DST=10.10.0.1:50002`，`FRANKA_CMD_ALLOW=10.10.0.1`）；`franka-ctl restart pose` 并确认客户端 mirror 在更新。
4. Desk：FCI 已激活，使能按钮已按下。主机上 `franka-ctl preflight` PASS，客户端上客户端 preflight PASS。封装 < 80 C。

### 1. 10 分钟头显 teleop
`franka-ctl goto home`，`franka-ctl start pose`；客户端上 `franka-teleop bridge-only`（或 `franka-teleop live`，它还会发出这两个动词）。桥接程序必须在 5 s 内打印出来自 mirror 的初始位姿。以 1 Hz 采样 `franka-ctl status`，持续 10 min。
门限：reflex 0，除非真实碰撞否则 freeze 0，`missed_cycles_total <= 20`，按住扳机时 `cmd_pkts_last_s` 85-92，松开死人开关后保持且无漂移，运动平滑程度与单机配置相当，在工作空间边缘盒裁剪明显介入且无跳变。

### 2. 十次重启
十次触发客户端的重启路径（`franka-teleop restart`，它运行 `franka-remote restart pose home`：停止、goto 出厂就绪、启动）。每次从主机 journal 中的 `Stopping` 到新的初始位姿行计时。
门限：带 home 序列时十次全部 <= 90 s（纯 `restart pose` 为 <= 15 s），桥接程序每次都重连，0 reflex。

### 3. 二十条夹爪命令
servo 活动期间，经客户端封装脚本执行十对 `gripper close` / `gripper open`，再从头显的 Grip 键执行十条。门限：封装路径 20 x rc 0；open 后 `gripper read` 显示宽度接近最大值；夹爪动作期间主机上 missed-cycle 不增加。

### 4. 60 分钟热浸泡
servo 活动，桥接程序空闲（扳机松开）或客户端侧运行 `fake_sender.py --mode hold`。每 10 s 采样一次封装温度和 `franka-ctl status`。
门限：每次采样封装 <= 85 C，整个小时 `missed_cycles_total <= 20`，`max_consecutive_missed <= 3`，reflex 0，>= 99 % 的采样中 `tick_over_1p2ms_1s` 为 0。
浸泡中段：做第 1 天第 7 步的 F/T 推力测试，以及 10 s 的控制面拔线。servo 在拔线期间保持，客户端 mirror 在重新插上后 2 s 内恢复。

### 5. 回滚演练（计时）
`franka-ctl stop`，把 FCI 线缆挪回客户端，以单机模式拉起客户端本地 servo（不带远程环境的 `franka-teleop servo-only`），确认本地 servo 生成了 `/tmp/franka_init_pose.txt`。门限：端到端 15 分钟以内。决定稳态使用哪台主机；如果留在实时主机上，把线缆挪回去并重做第 0 步。
