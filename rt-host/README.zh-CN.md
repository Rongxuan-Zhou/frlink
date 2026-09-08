[English](README.md) | 中文

# rt-host：Franka FR3 的专用 RT host / 实时主机

实时主机仓库（`alienware` 上的 `~/franka`）的导出。主机拥有机器人：它接着 FCI 线缆，在一颗隔离 CPU 上的 Docker 容器里运行 1 kHz 力矩 servo。它通过一根直连以太网线向客户端 PC 暴露三样东西。接口契约：[`../docs/INTERFACE.zh-CN.md`](../docs/INTERFACE.zh-CN.md)。设计理由与 servo 内部：[`../docs/GUIDE.zh-CN.md`](../docs/GUIDE.zh-CN.md) 第 2 节和第 3 节。

| 数据流 | 方向 | 传输 |
|---|---|---|
| 目标位姿 | 客户端 -> 主机 | UDP `10.10.0.2:50001`，128 字节 `O_T_EE` 数据报 |
| 机器人状态 | 主机 -> 客户端 | UDP `10.10.0.1:50002`，`FRST1` 数据报 |
| 控制动词 | 客户端 -> 主机 | ssh forced command `franka-ctl` |

会话期间主机上不运行其他任何东西；teleop bridge、相机、数据采集和策略推理都在客户端上。

## 目录内容

| 路径 | 内容 |
|---|---|
| `bin/` | `franka-ctl`（唯一的控制入口）、`franka-preflight`、`franka-run`（带 RT 容器配置的 docker run 封装）、`franka-servo-run` / `franka-servo-precheck`（servo 单元的 ExecStart / ExecStartPre）、`franka-py` |
| `host/` | `provision-rt.sh`、`verify-rt-host.sh`、`franka-rt-tune.sh`（每次开机的 CPU/IRQ/NIC 调优）、`franka-thermal-guard.sh`、`franka-docker-user.sh`，以及 `host/etc/`，包含安装到 `/etc` 下的每一个文件（GRUB 由脚本编辑，不随包提供） |
| `docker/` | `franka-rt:0.17.0-jazzy` 的 `Dockerfile`（Ubuntu 24.04 + ROS Jazzy pinocchio 快照）、`build-image.sh`、`build-inside.sh`（libfranka 0.17.0 和 teleop 二进制，在容器内构建）、ROS 快照 keyring |
| `teleop/` | servo（`cartesian_pose_servo.cpp`，6-DOF 笛卡尔阻抗，Franka Hand 负载模型）、运动辅助程序（`goto_home.cpp`、`gripper_cmd.cpp`、`fk_probe.cpp`）、`build.sh`，以及 `rt/`，即 header-only 的 RT/网络单元；标志与线上格式见 `teleop/rt/README.md` |
| `teleop/tests/` | `test_rt_units.cpp` + `rt_units/*.inc`（236 项检查，由 `build.sh` 运行）、`servo_net_stub.cpp`（不带机器人的 servo 网络侧） |
| `tests/` | `bench_net.sh`、`rt_bench.sh`（无机器人基准）、`state_sink.py`（带间隙门限的 FRST1 接收器）、`fake_sender.py`（128 字节位姿发送器：hold / circle）、`consumer_check.py`、`track_error.py`，以及机器人日检查清单 `robot_day1.md`、`robot_day2.md` |
| `franky_tools/` | 通过 `bin/franka-py` 运行的 franky 脚本：`goto_home.py`（`franka-ctl goto home` 实际运行的脚本）、负载标定、关节和夹爪摆动测试。直连 FCI：仅在没有 servo 活动时。 |
| `docs/` | `INTERFACE.md`（冻结的契约）、`RUNBOOK.md`（对接检查清单、故障表）、`ASSESSMENT-2026-09-02.md`（交接时的主机状态） |

未导出：编译产物、`tests/results/`、`libfranka/`（构建时克隆）、`local/`（构建输出）、virtualenv，以及 `keys/`（私钥从不离开主机）。

## 脚本假定的主机事实

- Ubuntu 22.04，内核 `6.8.0-124-generic`，通过 apt hold 和 GRUB 默认项固定。
- 内核命令行：`preempt=full isolcpus=domain,managed_irq,2,3 nohz_full=2,3 rcu_nocbs=2,3 irqaffinity=0-1,4-31 threadirqs nmi_watchdog=0 skew_tick=1`。
  CPU 2 运行 servo 控制线程；CPU 3 承担所有机器人网卡 IRQ（线程为 FIFO 85）。
- 机器人网卡 `enp110s0`（RTL8126，`r8126` DKMS，ASPM 和 EEE 关闭），`172.16.0.1/24`，机器人在 `172.16.0.2`。
- 控制面网卡 `frlink0`：一个 USB ASIX AX88179A 端口，在 `host/etc/systemd/network/10-franka-link.link` 中按 MAC 重命名，`10.10.0.2/24`，直连线到客户端，无交换机。
- Docker 镜像 `franka-rt:0.17.0-jazzy`；源码树 bind-mount 到 `/franka`，状态文件在 `/tmp/franka`。
- systemd 模板 `franka-servo@<mode>`；文档中的实例是 `franka-servo@pose`。它以编译默认值运行 `cartesian_pose_servo`（端口 50001，alpha 0.05，K_t 1000 N/m，K_r 80 Nm/rad，负载 0.25 kg / COM z 0.05 m = Franka Hand）。网络和 CPU 参数来自 `host/etc/franka/servo.env`。
- 安装路径在单元和脚本中硬编码为 `/home/rongxuan_zhou/franka`（见已知缺口）。

## 部署顺序

在新主机上按此顺序运行一次。每一步都有自己的验证。

1. `sudo host/provision-rt.sh`：apt hold（内核、nvidia）、`realtime` + `docker` 组、`limits.d/99-realtime.conf`、GRUB 默认项 + RT 命令行、unattended-upgrades 黑名单、`r8126` 模块选项 + initramfs、关闭 irqbalance、logind 合盖/空闲 + 屏蔽 sleep target、sysctl、tmpfiles。
2. 重启。
3. 从新登录的 shell 运行 `host/verify-rt-host.sh`：期望 `ALL PASS`（命令行、隔离 CPU、内核版本、`ulimit -Hr 99`、组、r8126 和 nvidia 的 DKMS、sleep 已屏蔽、tmpfiles）。
4. `docker/build-image.sh`：构建 `franka-rt:0.17.0-jazzy` 并记录 `last-build.txt`。
5. `teleop/build.sh`：以调用用户的身份在镜像内运行 `docker/build-inside.sh`：libfranka 0.17.0 装到 `local/`，teleop 目标（`cartesian_pose_servo`、`goto_home`、`gripper_cmd`、`fk_probe`）、`kIgnore` 通信测试、RT 单元测试（必须报告 236 checks passed）以及 `tests/servo_net_stub`。
6. 安装并启用 `franka-rt-tune.service`（+ NetworkManager dispatcher 钩子 `90-franka-rt-tune`）：CPU 2-3 上设 performance governor/EPP 并关闭深度 C-state，机器人网卡 IRQ 绑到 CPU 3，关闭 coalescing。只有每个硬性步骤都通过时它才写入 `/run/franka/rt-tune.ok`；没有该文件 servo precheck 拒绝启动。
7. 安装 `host/etc/nftables.conf`，`systemctl enable --now nftables`，以及 `franka-docker-user.service`（Docker 绝不能经机器人链路或客户端链路转发）。UDP 50001 只能从 `frlink0`（和 `lo`）、从 `cmd_allow` 集合内的地址访问。
8. 安装 `franka-servo@.service`、`franka-thermal-guard.{service,timer}` 和 `/etc/franka/servo.env`。servo 单元故意不在开机时启用；只有 `franka-ctl start pose` 会启动它。
9. `bin/franka-ctl preflight` 必须打印 `PREFLIGHT PASS`。然后运行 `tests/rt_bench.sh` 和 `tests/bench_net.sh`（无机器人），再做 `tests/` 里的机器人检查清单。

## `franka-ctl` 动词

`franka-ctl` 是唯一的控制入口，本地调用和作为 ssh forced command 都是。所有触碰机器人的操作都要先拿 `flock -n /run/lock/franka-robot.lock`；`gripper` 有自己的锁，可以与活动的 servo 并行运行。退出码透传；`64` = 用法错误 / 被拒绝的动词，`75` = 忙（锁被持有或有 servo 活动）。

| 动词 | 作用 |
|---|---|
| `status` | 单元 active/inactive、link/fci 载波、最新的 `franka_link.txt` 计数器、封装温度（镜像是否存在由 `preflight` 报告） |
| `preflight` | isolcpus、rtprio 99/memlock、IRQ 亲和 enp110s0-* = cpu3、NM profile 已启用、nftables 已加载、timesyncd、镜像、机器人 ping |
| `start pose` | `systemctl start franka-servo@pose` -> docker run --privileged --network host --ulimit rtprio=99 ... 以编译默认值运行 `cartesian_pose_servo` |
| `stop` | `systemctl stop franka-servo@*`（SIGTERM，10 s 宽限） |
| `restart pose [home]` | stop + start；带 `home` 时中间把机械臂驱动到出厂就绪位姿（客户端的 restart 路径调用的就是它） |
| `goto home` | `franky_tools/goto_home.py`（经 `bin/franka-py`）以速度 0.10 前往出厂就绪位姿：从桌面附近启动时先抬到离桌高度，并在运动前用 FK 检查关节路径（`franky_helpers.path_min_z`）。有 servo 活动时拒绝执行。 |
| `gripper open\|close\|width w\|grasp w\|homing\|read` | 在 Franka Hand 上运行 `gripper_cmd`；rc 0 成功，4 为 franka::Exception；`read` 打印 JSON。可在 servo 活动时运行（独立锁）。 |
| `echo` | 运行一次 `echo_robot_state`。有 servo 活动时拒绝执行。 |
| `comm-test --yes` | `communication_test`（kIgnore 构建），自动喂入 Enter；**会移动机械臂**，以速度 0.5 到出厂就绪位姿，然后 10 s 零力矩；期望 Avg >= 0.99 |

通过 ssh 时，`franka-ctl` 读取 `SSH_ORIGINAL_COMMAND`，拒绝任何含 `[A-Za-z0-9._:=-]` 之外字符的 token，所以客户端无法通过 forced key 串联 shell 命令。

## 授权新的客户端密钥

每台客户端 PC 都有自己的 ed25519 密钥对；只有公钥放到主机上。在主机（用户 `rongxuan_zhou`）的 `~/.ssh/authorized_keys` 里追加一行：

```
command="/home/rongxuan_zhou/franka/bin/franka-ctl",no-port-forwarding,no-X11-forwarding,no-agent-forwarding,no-pty,from="10.10.0.0/24" ssh-ed25519 AAAA...your-public-key... client-name
```

- `command=` 把该密钥变成 forced command：客户端 `ssh` 后面跟的内容成为 `SSH_ORIGINAL_COMMAND`，被解析为 `franka-ctl` 动词。该密钥无法打开 shell。
- `from="10.10.0.0/24"` 把密钥限制在直连线子网。只有为了从别处进行有意的管理访问才添加另一个 CIDR；nftables 也只在 `frlink0` 上接受来自 `10.10.0.0/24` 的 TCP 22。
- 从客户端测试：`ssh -i <key> -o IdentitiesOnly=yes rongxuan_zhou@10.10.0.2 status` 必须打印状态块；`ssh ... 'status; id'` 必须被拒绝并以 64 退出。

主机上 `keys/` 下的密钥文件是私有的，不属于本导出。

## 同一时间只有一个客户端：来访 PC 必须是 10.10.0.1

servo 向一个目的地发布状态，只接受来自一个源地址的命令，两者都固定在 `/etc/franka/servo.env` 里：

```
FRANKA_CMD_BIND=10.10.0.2
FRANKA_CMD_ALLOW=10.10.0.1
FRANKA_STATE_DST=10.10.0.1:50002
```

nftables 的 `cmd_allow` 集合持有同一个地址。没有发现机制，没有多客户端扇出：来访 PC 插到 `frlink0`，在自己的网卡上设置 `10.10.0.1/24`，就是客户端。同一地址内的发送者 latch 见 [`../docs/GUIDE.zh-CN.md`](../docs/GUIDE.zh-CN.md) 第 2.2 节。要在主机自身上运行测试客户端（第 1 天检查清单就是这么做的），在那个会话里覆盖这三个变量，事后恢复文件；不要提交覆盖后的版本。

位姿 servo 不接受任务参数：`servo.env` 中的 `FRANKA_POSE_ARGS` 为空，编译默认值就是“脚本假定的主机事实”一节列出的那些。它把机器人的碰撞行为设为 30 N / 30 Nm，并在外力达到 35 N / 20 Nm 持续 1500 ms 时冻结。

## 已知缺口

- 安装前缀 `/home/rongxuan_zhou/franka` 硬编码在 `bin/franka-ctl`、`bin/franka-run`、`bin/franka-servo-run`、`bin/franka-preflight`、`bin/franka-servo-precheck`、`host/provision-rt.sh` 和 systemd 单元里。换用户意味着全局搜索替换，或者引入 `FRANKA_ROOT` 变量（尚未做）。
- `host/provision-rt.sh` 内嵌了主机根文件系统的 UUID 和 GRUB 默认项的精确内核版本；在别处运行前两者都要改。
- `10-franka-link.link` 中的 MAC 地址和 udev 规则 `80-franka-ax88179.rules` 是本机 dock 端口专用的。
- `teleop/notes/` 和 `teleop/scripts/`（以前放在主机上的客户端 bridge 脚本）被有意排除；客户端包已取代它们。
- 主机仓库还包含任务专用的 servo 变体及其 `franka-ctl` 模式；它们未导出。这里的一切都是带 Franka Hand 的常规 6-DOF 位姿 servo，这一省略不影响 `../docs/INTERFACE.md`。
