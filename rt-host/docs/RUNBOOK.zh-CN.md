[English](RUNBOOK.md) | 中文

# alienware 上的 FR3：runbook

RT host / 实时主机的检查清单和故障表。动词：[`../README.zh-CN.md`](../README.zh-CN.md#franka-ctl-动词)。为什么这样搭建：[`../../docs/GUIDE.zh-CN.md`](../../docs/GUIDE.zh-CN.md)。

## 对接检查清单（每次会话，3 分钟）
1. 已接交流电（`cat /sys/class/power_supply/AC*/online` -> 1）。用电池时 CPU 频率下降，会丢周期。
2. 两根线：FCI 以太网 -> enp110s0（内置 RJ45，r8126）；USB dock 端口 frlink0（ASIX AX88179A，MAC 00:0a:cd:48:1e:dd）-> 直连线 -> rog eno1（私有链路 10.10.0.0/24，无交换机）。**绝不要接反**：nftables 只允许 enp110s0 上的 172.16.0.2 和链路上的私有子网。
3. 盖子：只有当 `systemctl is-enabled sleep.target suspend.target hibernate.target` -> 三者都是 masked，且 `systemd-analyze cat-config systemd/logind.conf | grep HandleLidSwitch` -> ignore 时，合盖才安全。否则保持打开。
4. `~/franka/bin/franka-ctl preflight` -> exit 0。**任何一行 FAIL 都不要启动 servo。**
5. Desk（https://172.16.0.2/desk/）：解锁关节，Activate FCI，按下使能按钮。机器人每次开机之后都需要。

## 每日启动 / 停止
- 启动：`franka-ctl preflight && franka-ctl goto home && franka-ctl start pose`。`franka-ctl status` 必须显示 servo 活动以及 cmd_age_ms/missed 计数器。
- rog 侧：mirror 在运行（`/tmp/franka_current_ee.txt` 的 mtime 在推进），`FRANKA_SERVO_HOST=10.10.0.2`，然后照旧 bridge / 采集 / 部署。
- 停止：`franka-ctl stop`（SIGTERM -> MotionFinished -> 约 1 s 内以 0 退出）。离开时再到 Desk -> 锁定关节。
- **servo 运行期间不得有其他任何东西与 172.16.0.2 通信**（不 `echo`，不 `goto`，不 `comm-test`；夹爪的 TCP 路径是独立的，允许）。

## 故障表
| 症状 | 可能原因 | 处理 |
|---|---|---|
| `preflight` FAIL "enp110s0 missing" / 重启后没有 172.16.0.1 | 内核更新重建/移除了 r8126 DKMS | `dkms status`；`sudo dkms autoinstall`；`sudo modprobe r8126`；如果编不过，从 GRUB 启动上一个内核（`apt-mark showhold` 本应阻止这种情况）。备用网卡：Thunderbolt 上的 Intel I225，重新指向 `franka-fci`（`nmcli con modify franka-fci connection.interface-name <new>`）。 |
| 遥操作时机械臂原地冻结；`status` 显示 cmd_age_ms > 200 | hold 已触发：没有命令包（bridge dead-man、发送者崩溃、FRANKA_SERVO_HOST 错误、allowlist 丢弃） | `status`：cmd_drop_allow 上升 -> 发送者 IP 不在 CMD_ALLOW；cmd_drop_latch 上升 -> 另一个发送者持有 latch；cmd_pkts_last_s = 0 -> 发送者已死，重启它。hold 是安全的；力矩控制继续。 |
| rog 脚本报 EE 文件 STALE / ee_ok false / 录制器丢帧 | rog mirror 没运行、链路断开或 state-dst 错误 | alienware `status` -> 发布器计数器；两端 `ip -br addr`；`ping 10.10.0.1`；重启 rog mirror；rog 上 `ls --full-time /tmp/franka_current_ee.txt` 必须每 50 ms 推进。 |
| comm-test Avg < 0.99 | 网卡 coalescing/IRQ 在错误的 CPU 上、热降频、其他 RT 负载、FCI 路径里有交换机 | `ethtool -c enp110s0`（rx-usecs 0），`/proc/interrupts` 中 enp110s0-* 只在 cpu3，`sensors` < 85 C，`ps -eLo cls,rtprio,psr,comm | grep FF`，机器人直连线；重试 3 次；仍 < 0.99 则改用 I225 网卡。 |
| servo 日志反复 reflex，或机器人灯红/黄 | 碰撞或关节限位 reflex；按下了急停 = 需要 Desk | 释放急停，Desk -> 确认错误，若 FCI 掉了就重新 Activate FCI；`franka-ctl stop`，`goto home`，`start pose`。碰撞阈值（30 N/30 Nm）低于 35 N 的软件冻结阈值。 |
| `status` 温度中位数 >= 85 C，丢周期增长，tick_max_us_1s > 800 | 热降频 / 硬件停顿（约 100 C 时见过 0.3 到 0.7 ms 的 hwlat 事件） | 停掉 alienware 上的 GPU/CPU 任务，`docker stats` 查找游离容器，垫高笔记本，检查风扇；考虑 `intel_pstate/no_turbo=1`；这台机器做实时主机期间绝不要在上面训练。 |
| 新发送者被忽略，cmd_drop_latch 上升，latched_sender 是旧的 ip:port | latch 被已死/挂起的发送者持有（最后一包之后 1 s 释放） | 在 rog 上杀掉旧发送者；等 1 s；若仍然如此则 `franka-ctl restart pose`。 |
| `start` 失败：`Unable to find image 'franka-rt:0.17.0-jazzy'` | 镜像被清理或 Docker 重装 | `docker images | grep franka-rt`；用 `~/franka/docker/build-image.sh` 重建（然后 `teleop/build.sh` 编译二进制）。 |
| 新 shell 里 `ulimit -r` 打印 0 | 不在 `realtime` 组 / limits.d 缺失 / GUI 会话早于改动 / Tailscale SSH（绕过 PAM，limits.d 从不生效；从这种登录启动的 tmux server 继承 0） | `groups`，`cat /etc/security/limits.d/99-realtime.conf`，重启；通过 OpenSSH（端口 22）登录，或用 `sudo prlimit --pid $$ --rtprio=99:99 --memlock=unlimited:unlimited` 修复当前 shell。容器内 servo 不受影响（--ulimit rtprio=99）。 |
| 链路线被拔 / USB dock 重新枚举 | NM 丢掉了 10.10.0.2；端口失去了 frlink0 名字（/etc/systemd/network/10-franka-link.link 中的 MAC 匹配） | `nmcli con up franka-link`；`ip -br link`；若被重命名，`nmcli con modify franka-link connection.interface-name <name>`。FCI 侧不受影响。 |

## 结果存放位置
`~/franka/tests/results/<date>/...`（bench_net、rt_bench、robot_day1、robot_day2）。内核、驱动、Docker 或 servo 有任何改动后，重新运行 `tests/bench_net.sh` 和 `tests/rt_bench.sh`。
隔离核上的快速 RT 健全性检查：`sudo taskset -c 2 cyclictest -m -p 80 -t 1 -i 1000 -l 30000 -q`（2026-09-03 隔离后基线：Max 30 µs）。`cyclictest -a 2` 会以 EINVAL 失败，因为 isolcpus 把 cpu2 从继承的亲和掩码中移除了。
