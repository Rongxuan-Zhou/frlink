[English](GUIDE.md) | 中文

# 从你自己的电脑使用 FR3 + 实时主机组合

本指南讲的是从你自己的 Linux PC 驱动位于实时主机（RT host，一台 Alienware m18 笔记本）之后的 Franka FR3：Quest teleop、episode 录制，或者你自己的控制器发送目标位姿。要配置一台新的实时主机：[`../rt-host/README.zh-CN.md`](../rt-host/README.zh-CN.md)。

目录

1. [你连接的是什么](#1-你连接的是什么)
2. [原理：为什么要两台机器，它们如何通信](#2-原理为什么要两台机器它们如何通信)
3. [实时主机，代码层面](#3-实时主机代码层面)
4. [客户端软件包，代码层面](#4-客户端软件包代码层面)
5. [运行一次会话，逐步说明](#5-运行一次会话逐步说明)
6. [记录数据](#6-记录数据)
7. [编写你自己的发送端或消费端](#7-编写你自己的发送端或消费端)
8. [可能出什么问题，以及它的表现](#8-可能出什么问题以及它的表现)
9. [你可以预期的数字](#9-你可以预期的数字)
10. [软件栈与库](#10-软件栈与库)

---

## 1. 你连接的是什么

```
                 FCI 网线（1 kHz，硬实时）                  直连网线（控制平面，10.10.0.0/24）
 Franka FR3 <=================================> RT host <===============================================> 你的 PC
 172.16.0.2      内置网卡 enp110s0            Alienware      USB 口 frlink0 10.10.0.2          网卡 10.10.0.1
                                              m18 R2
```

由实验室提供：

- FR3 机械臂与 Franka Hand，系统镜像 5.7.2，带 FCI 许可。它只和实时主机通信。它的网页控制台（“Desk”，`https://172.16.0.2/desk/`）只能从实时主机访问；见 [`DESK.zh-CN.md`](DESK.zh-CN.md)。
- 实时主机：Ubuntu 22.04，内核固定在 6.8，CPU 2-3 隔离给控制线程，servo 跑在 systemd 管理的 Docker 镜像（`franka-rt:0.17.0-jazzy`，libfranka 0.17.0）里。一个以太网口，同一时间只接一个客户端，外加一条仅限 ssh 密钥的控制通道。
- 两台机器之间冻结的接口契约，[`INTERFACE.zh-CN.md`](INTERFACE.zh-CN.md)。本仓库实现了它。

你自带：

- 一台有空闲以太网口的 Linux PC（NetworkManager、systemd 用户会话、Python 3）。
- 一根网线。你的网卡设为 `10.10.0.1/24`；主机是 `10.10.0.2`。
- 一个 ssh 公钥。主机管理员把它加进主机的 `authorized_keys`，附带一条只允许 `franka-ctl` 动词的 forced command。
- 可选：一台 Meta Quest（teleop）、Intel RealSense 相机（录制）。

你要安装的东西都在 [`../client/`](../client/README.zh-CN.md)。

## 2. 原理：为什么要两台机器，它们如何通信

### 2.1 为什么 servo 不在你的 PC 上

libfranka 的力矩接口要求每 1 ms 一条力矩命令，连续漏掉 20 次就放弃。桌面内核每分钟都会有几次停顿几毫秒（在原来那台工作站上实测：每个核心都有约 6 ms 的停顿）。实时主机消除了这种波动：`preempt=full`、`isolcpus=2,3`、网卡中断绑定到 cpu 3、隔离核心上限制 C-state、控制线程绑定到 cpu 2 并 `mlockall`。隔离核心上实测的唤醒抖动：p99 = 11 us，p99.9 = 49 us。你的 PC 只有软性时限：每约 11 ms 发一个目标位姿，每 50 ms 读一次状态。

### 2.2 三条数据流

| 流 | 方向 | 传输 | 内容 |
|---|---|---|---|
| A. 命令 | PC -> 主机 | UDP `10.10.0.2:50001` | 一个数据报 = 128 字节 = 16 个 float64（小端），列主序 4x4 齐次 `O_T_EE` 目标 |
| B. 状态 | 主机 -> PC | UDP `10.10.0.1:50002` | `FRST1 <seq> <epoch_ns> <t_real_ns> <t_mono_ns> <name>\n<body>`；body 是一个状态文件的文本 |
| C. 控制 | PC -> 主机 | ssh forced command | `franka-ctl status / preflight / start pose / stop / restart pose [home] / goto home / gripper ... / echo / comm-test --yes` |

流 A 没有序号，也没有应答。servo 只接受来自白名单源 IP（`10.10.0.1`）且长度恰好为 128 字节的数据报。它会 latch 第一个成功送达的 (ip, port)，之后丢弃其他所有发送端，直到被 latch 的那个静默满 1 s。

流 B 是单向流，包含五种命名消息，各以固定速率发送：

| 名称 | token 数 | 速率 | 内容 |
|---|---|---|---|
| `franka_init_pose.txt` | 16 | servo 启动时一次，之后每 1 s 重发 | servo 接管那一刻的 `O_T_EE` |
| `franka_current_ee.txt` | 16 | 20 Hz | 实时 `O_T_EE` |
| `franka_wrench.txt` | 6 | 20 Hz | `O_F_ext_hat_K`（Fx Fy Fz Tx Ty Tz） |
| `franka_joint_state.txt` | 28 | 10 Hz | q[7] dq[7] tau_J[7] tau_ext_hat_filtered[7] |
| `franka_link.txt` | 13 个 key=value | 5 Hz | 链路与健康计数器（见 4.1） |

`epoch_ns` 是 servo 进程的墙钟启动时间；epoch 变了就意味着换了一个 servo 实例。body 是纯 ASCII，采用 C++ 流的默认格式（6 位有效数字），任何语言都能用 `float()` 读取。

流 C 的存在是为了让力矩控制器永远不会意外启动：主机只运行上面这些动词，每个动词都对机器人加文件锁，而且所有会让机械臂动起来的动词在 servo 活动期间一律拒绝执行。

### 2.3 安全包络（无论你发什么，servo 都会做的事）

- 200 ms 没有被接受的命令就 hold：每个 tick 都把目标贴到当前位姿，于是机械臂停下并保持柔顺。发送端重启后立即恢复控制。
- 力/力矩超过 35 N 或 20 Nm 就 freeze：目标冻结 1.5 s，期间命令被丢弃（计入 accepted，但不传给控制器）。
- 力矩上限 0.85 x [87 87 87 87 12 12 12] Nm，libfranka 自带的限速器，以及机器人上设置的碰撞 reflex 阈值 30 N / 30 Nm。
- reflex 恢复：reflex（碰撞、关节限位、速度）会在 `robot.control()` 内部抛出异常。servo 执行 `automaticErrorRecovery`，重新读取位姿，冻结 2 s 后重新进入控制，最多 50 次。`franka_link.txt` 里的 `reflex_count` 和 `recovering` 会反映出来。
- 网络防护：50001 端口上的白名单与 latch（2.2），主机 nftables 只从客户端口接受 50001，以及 servo 启动前的 precheck（3.2）。
- 客户端上 bridge（`franka-teleop`）的防护：工作空间盒，每帧步长上限 5.5 mm（90 Hz 下 0.5 m/s），旋转缩放 0.5，dead-man 扳机。

### 2.4 为什么客户端要写文件

客户端上的每个消费端（bridge、录制程序、你的脚本）都从 `/tmp/franka_*.txt` 读机器人状态，而不是从网络读；`franka_state_mirror` 守护进程从流 B 写这些文件，用原子重命名。存活判断靠文件 mtime：`franka_current_ee.txt` 超过 250 ms 没更新，就说明 servo 挂了或链路静默。状态读取程序分不清单机和双机方案，十行代码就能写完。

### 2.5 时钟

状态数据报带的是主机的墙钟。episode 要把相机帧（客户端时钟）和机器人位姿（主机时钟）对齐，动作标签又是相邻位姿之差，所以两个时钟必须在 10 ms 以内一致。两台机器都跑 NTP；mirror 持续测量偏差，`franka-client-preflight` 会报告它。

## 3. 实时主机，代码层面

所有路径都在 [`../rt-host/`](../rt-host/) 下。访客改不了其中任何东西。

### 3.1 `bin/franka-ctl`：唯一入口

```
verb -> [refuse_if_servo] -> [with_robot_lock: flock -n /run/lock/franka-robot.lock] -> action
```

- `start pose`：`sudo systemctl start franka-servo@pose`（sudoers 恰好只允许这些 unit），等 2 s，确认 `active`。退出码：0，64（动词或 token 非法；forced command 拒绝任何不匹配 `[A-Za-z0-9._:=-]+` 的参数），75（忙：锁被占用或 servo 活动中）。
- `stop`：`systemctl stop` -> SIGTERM -> `MotionFinished` -> 约 1 s 内干净退出。
- `restart pose home`：停止、回 home 序列、启动，全程持有同一把机器人锁。回 home 失败（例如一次 reflex）退出码为 1，servo 保持停止。
- `goto home`：franky 版本（`bin/franka-py franky_tools/goto_home.py`）。若末端靠近桌面，先把它抬到 0.15 m，再用正运动学检查关节路径（`franky_helpers.path_min_z`），路径低于 0.105 m 就拒绝执行。
- `gripper <argv>`：在一次性容器里运行 `gripper_cmd`，走夹爪自己的 TCP 通道（端口 1338），所以 servo 运行时也允许。
- `echo`：输出一份 JSON 机器人状态；servo 活动时拒绝（它会抢走 FCI 通道）。

### 3.2 `franka-servo@.service`：servo 如何启动

`ExecStartPre=bin/franka-servo-precheck` 在以下情况拒绝启动：`/run/franka/rt-tune.ok` 缺失（RT 调优没跑）、nftables 表未加载、没接交流电、10 s 中位数封装温度 >= 85 C、另一个机器人容器在运行、绑定地址未配置，或镜像/二进制缺失。`ExecStart` = `systemd-inhibit`（禁止休眠/合盖）+ `bin/franka-servo-run` =

```
docker run --rm --init --privileged --network host --ipc host --ulimit rtprio=99 --ulimit memlock=-1
    franka-rt:0.17.0-jazzy /franka/teleop/cartesian_pose_servo 172.16.0.2
    --cmd-bind 10.10.0.2 --cmd-allow 10.10.0.1 --state-dst 10.10.0.1:50002
    --rt-cpu 2 --aux-cpus 4-15 --state-files 1 --state-dir /tmp/franka
```

日志：主机上的 `/tmp/franka/servo-pose.log`；计数器在 `/tmp/franka/franka_link.txt`。

### 3.3 `teleop/cartesian_pose_servo.cpp`：控制器

一个进程，七个线程：

| 线程 | cpu | 工作 |
|---|---|---|
| main（控制） | 2，SCHED_FIFO 99（由 libfranka 设置） | 1 kHz 的 `robot.control(torque_cb)` |
| `udp_cmd` | 4-15 | 带 100 ms 超时的 `recvfrom` -> `CmdFilter` -> `TripleBuffer<target>` |
| `ee_writer`、`wrench_writer`、`joint_writer` | 4-15 | 20/20/10 Hz：格式化最新样本，写文件，发数据报 |
| `init_resend`、`link_writer` | 4-15 | 1 Hz 重发初始位姿，5 Hz 链路计数器 |

1 kHz 回调不做分配、I/O 或加锁。每个 tick：

1. `tick_stats.on_tick()`（间隔检测 -> `missed_cycles_total`、`tick_max_us_1s`）。
2. 从无锁 `TripleBuffer` 读最新目标（写入方：udp 线程）。200 ms 没有被接受的数据包，或处于 freeze 期间，一个 tick 局部的覆盖值会用当前位姿替代目标；新数据包到达即取消。
3. 对目标做两级低通滤波（平移 EMA alpha 0.10 再 0.05；旋转用带半球修正的四元数 slerp）。
4. 笛卡尔阻抗：`tau = J^T (-K e - D J dq) + coriolis`，K_t = 1000 N/m，K_r = 80 Nm/rad，阻尼按 5 kg / 0.3 kg m^2 的等效质量临界匹配。接近接触时（|F| 在 10 到 25 N 之间）刚度逐渐降到 25 %；接近奇异（可操作度 w < 0.015）时刚度被缩小；零空间项把关节推离限位（0.5 rad 余量）。
5. 钳到力矩上限，`franka::limitRate`，返回。收到 SIGTERM 后返回 `MotionFinished`。

网络与 RT 管道都是 `teleop/rt/` 下的 header-only 代码：`udp_cmd.hpp`（socket + 白名单/长度/latch 过滤）、`state_pub.hpp`（文件和数据报共用一个格式化器，所以字节完全一致）、`triple_buffer.hpp`、`rt_setup.hpp`（mlockall、栈预触、亲和性）、`writers.hpp`。`tests/servo_net_stub.cpp` 是同一程序去掉 libfranka 的版本，用来在没有机器人时测试客户端。

## 4. 客户端软件包，代码层面

所有路径都在 [`../client/`](../client/) 下。一个配置文件 `config.env`（从 `config.env.example` 复制）：主机地址和用户、密钥路径、你的 IP 和网卡名、bridge 用的 Python 解释器。每个脚本都从自身位置推导仓库根目录；没有任何硬编码。

### 4.1 `bin/franka_state_mirror`：流 B 落成文件

一个只用标准库的 Python 守护进程，由用户 unit `franka-state-mirror` 运行（`install.sh` 安装）：

```
recvfrom 0.0.0.0:50002 -> 源地址必须是 $FRANKA_SERVO_HOST
  -> 头部：6 个 token，"FRST1"，四个整数             （否则 drop_hdr）
  -> name 必须是五个已知名称之一                      （否则 drop_name）
  -> body：N 个 float，或按顺序排列的 13 个 link key  （否则 drop_body）
  -> 该 name 的 seq 不得倒退                          （否则 drop_reorder）
  -> epoch 变了？unlink franka_init_pose.txt，重置各 name 的 seq，记录 "EPOCH CHANGE"
  -> 写 /tmp/.franka_<name>.tmp，os.replace() -> /tmp/<name>
```

流停止时不删除任何东西（消费端看 mtime）。`franka_init_pose.txt` 每个 epoch 只写一次，所以它的 mtime 就表示“这个 servo 实例启动了”（4.4 依赖这一点）。每秒向 `/tmp/franka_mirror.txt` 写一行状态：`epoch seq src age_ms skew_ms rx drop_src drop_hdr drop_name drop_body drop_reorder ...`。`--check` 是 preflight 使用的健康探针（状态行不超过 2 s，EE 文件不超过 250 ms）。

`franka_link.txt` 的 key，按顺序：`cmd_age_ms`（第一条被接受的命令之前为 -1）、`cmd_pkts_last_s`、`cmd_drop_size`、`cmd_drop_allow`、`cmd_drop_latch`、`latched_sender`、`missed_cycles_total`、`max_consecutive_missed`、`freeze`、`recovering`、`tick_over_1p2ms_1s`、`tick_max_us_1s`、`reflex_count`。

### 4.2 `bin/franka-remote`：流 C

在 `timeout` 之下执行 `ssh -i $FRANKA_CTL_KEY -o BatchMode=yes -o ConnectTimeout=3 user@host <verb...>`（status/gripper/echo 为 20 s，start/stop 为 45 s，goto/restart 为 120 s，comm-test 为 90 s）。退出码原样透传：75（忙）、255（无链路）、124（超时）。

### 4.3 `bin/franka-fci-shim`：旧的二进制名字

`bin/gripper_cmd`、`bin/goto_home`、`bin/echo_robot_state` 是指向同一个脚本的符号链接，它把经典的参数形式（`gripper_cmd <ip> close`）映射成 `franka-remote gripper close`，这样 bridge 调用 `gripper_cmd` 的方式和本地机器人时代一样。主机不提供的功能（自定义关节目标）以 64 退出，而不是被映射成别的运动。

### 4.4 `bin/franka-teleop`：编排器

```
live    = servo_start -> wait_state_ready -> bridge_start
restart = bridge_kill -> franka-remote restart pose home (retry only on 75) -> wait_state_ready 15 -> bridge_start
stop    = bridge_kill -> franka-remote stop
```

`wait_state_ready` 取代了“sleep 5”：mirror 必须报告新的 epoch，`franka_init_pose.txt` 必须有 16 个 token，`franka_current_ee.txt` 必须不超过 250 ms。启动 servo 之前它会删掉旧的 `franka_init_pose.txt`，所以 bridge 绝不会读到上一个 servo 的位姿。工作空间盒、缩放和步长上限是脚本顶部的参数（可用 `FRANKA_WS_*`、`FRANKA_SCALE_*`、`FRANKA_MAX_STEP` 环境变量覆盖）。

### 4.5 `teleop/02_webxr_to_franka.py`：bridge

- 通过 `teleop` pip 包在 4443 端口以 HTTPS 提供 WebXR 页面（`frontend_swapped/`），每个头显帧（约 90 Hz）收一条 JSON 消息：手柄位置/朝向、`move`（Trigger 按住）、`gripper`（Grip 切换）、scale。
- 启动时读取 `franka_init_pose.txt`（最多等 5 s）。第一个按住 Trigger 时的手柄位姿是手的原点。之后每个位姿都是相对该原点的增量，绕机器人基座 z 轴镜像（`_R_MIRROR_Z`：你的“朝机器人”对面对你的机械臂来说是“朝你”），再缩放（平移 1.0，旋转 0.5），然后叠加到初始位姿上：`p = p_init + M delta_p`，`R = R_init M delta_R M`。
- 防护按顺序：工作空间盒裁剪、每帧步长钳位（5.5 mm）、旋转步长钳位（0.05 rad）、靠近盒边的减速区。裁剪之后手的原点会重新锚定，这样你回来时机械臂不会跳。
- dead-man：松开 Trigger -> 重发最后一个目标（servo 会 hold 住它）；bridge 挂掉的情况由 servo 自己的 200 ms 规则兜底。
- 数据包：`struct.pack("16d", *T.flatten(order="F"))` 发到 `--udp-host:50001`。
- 夹爪：Grip 的边沿触发会 spawn `gripper_cmd <ip> open|close`（经 shim，也就是 `franka-remote`）。
- PC 上同一时间只有一个发送端：`franka_sender_lock.acquire_sender_lock()` 对 `/tmp/franka_sender.lock` 加 `flock`；第二个活动发送端以 75 退出。

### 4.6 `record/`：episode 落成 HDF5

- `cameras/grabber.py`：按 `config/cameras.yaml`（序列号 -> 角色）为每台 RealSense 相机开一个线程，640x480 @ 30 fps 彩色（+ 对齐的深度），自动曝光收敛后锁定，每帧保留硬件时间戳。
- `state_reader.py`：`make_state_fn()` 返回一个函数，它读取 `/tmp/franka_current_ee.txt`（可选力/力矩和关节），应用 250 ms 过期规则，返回 `{"ee_pose": (16,), "ee_ok": bool, ...}`。
- `record_episodes.py`：键盘驱动（`s` 开始，`e` 结束，`d` 丢弃，`q` 退出）或 `--auto-end-secs`；以 20 Hz 轮询状态，缓存帧，在后台线程里写每个 episode。每个文件的布局：

```
observations/<role>/image            (N, 480, 640, 3) uint8
observations/<role>/hw_timestamp_ms  (N,) float64      相机硬件时钟
observations/<role>/wall_timestamp_s (N,) float64      收到帧时的客户端墙钟
state/ee_pose                        (M, 16) float32   列主序 O_T_EE，来自 mirror
state/ee_ok                          (M,) bool         False = 过期样本（全零）
state/_t                             (M,) float64      轮询时的客户端墙钟
```

按墙钟对齐（两者都在客户端），把 `ee_ok == False` 的样本当作空洞。

## 5. 运行一次会话，逐步说明

### 5.1 在你的 PC 上一次性设置

```
git clone <this repo>; cd client
cp config.env.example config.env; $EDITOR config.env      # FRANKA_CTL_USER, FRANKA_LINK_IFACE, FRANKA_PY
nmcli con add type ethernet ifname <your NIC> con-name franka-link ipv4.method manual \
    ipv4.addresses 10.10.0.1/24 ipv4.never-default yes ipv6.method disabled connection.autoconnect yes
./install.sh          # bin -> ~/.local/bin, ssh key + config block, mirror user unit; prints your authorized_keys line
pip install -r requirements-teleop.txt      # into $FRANKA_PY's environment
pip install -r requirements-record.txt      # only if you record
```

把打印出的 `authorized_keys` 那一行交给主机管理员。在它被装上之前，`franka-remote status` 会以 255 退出。

### 5.2 每次会话

```
source config.env
franka-client-preflight        # every line PASS or an explained WARN
```
让管理员（或者你自己在主机屏幕上）操作：Desk -> 解锁关节 -> Activate FCI -> 按下使能按钮。机器人每次重新上电后都要做；你 PC 上的任何东西都做不了这一步。Desk 步骤、手动引导和错误恢复：[`DESK.zh-CN.md`](DESK.zh-CN.md)。

```
franka-remote status           # franka-servo@pose inactive, fci link up
franka-remote goto home        # arm moves to factory-ready (~10 s), user stop in hand
franka-teleop live             # servo on the host, state ready, bridge up (~7 s)
franka-teleop status
```

### 5.3 Teleop

在 Quest 上打开 `https://<your PC IP>:4443/`（与你的 PC 同一网络，或者你自己搭的隧道；页面用自签名证书，接受一次即可）。细节和手柄按键映射：[`../client/teleop/README.zh-CN.md`](../client/teleop/README.zh-CN.md)。

### 5.4 头显中的操作流程

1. **按 Enter VR 之前，站到机器人正前方，面对它。** WebXR 坐标系在那一刻被捕获，之后所有手部动作都在这个坐标系里解释。
2. 之后你可以走动，到侧面或者绕到机械臂同一侧的后方。映射不会随你旋转；机械臂始终按标定好的方向运动。
3. 按住 Trigger 移动；走动之前先松开。按 Grip 切换夹爪。
4. **结束时先在网页里按 Exit VR**，再在 PC 上执行 `franka-teleop stop`，如果要离开机械臂再执行 `franka-remote goto home`。

### 5.5 停止

```
franka-teleop stop             # bridge, then the servo (clean MotionFinished, ~1 s)
franka-remote goto home
```
如果 bridge 或头显在会话中途乱了，用 `franka-teleop restart`（停止、回 home、启动、再起 bridge，约 8 s）。

## 6. 记录数据

```
python3 record/state_reader.py --selftest --secs 3                  # expect ee_ok 60/60 while the servo runs
python3 record/record_episodes.py --out-dir ~/datasets/<task> --prefix <task> [--with-wrench --with-joints]
```
在 `franka-teleop live` 运行时于第二个终端里执行。相机按序列号在 `record/config/cameras.yaml` 里查找；用 `rs-enumerate-devices | grep Serial` 找到你的。状态路径已死时录制程序拒绝启动（纯相机测试可用 `--allow-dead-state` 覆盖），某个 episode 的 `ee_ok` 比例低于 95 % 时会警告。

## 7. 编写你自己的发送端或消费端

发送端（任何语言；这里用 Python 演示）。读当前位姿，以 10-100 Hz 发送 128 字节目标；暂停超过 200 ms 机械臂就 hold：

```python
import socket, struct, time, numpy as np
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("10.10.0.1", 0))                       # source must be 10.10.0.1 (allow-list)
T = np.array([float(x) for x in open("/tmp/franka_current_ee.txt").read().split()]).reshape(4, 4, order="F")
for k in range(1000):                            # 10 s at 100 Hz
    T[2, 3] = T[2, 3] + 0.0001                   # creep up 0.1 mm per step (10 mm/s)
    sock.sendto(struct.pack("16d", *T.flatten(order="F")), ("10.10.0.2", 50001))
    time.sleep(0.01)
```
如果和 bridge 共用这台 PC，先拿发送端锁（`teleop/franka_sender_lock.py`）。步长要小：servo 会滤波并钳制力矩，但不会钳制你的目标；10 cm 的跳变就是 10 cm 的跳变。在 `franka_link.txt` 里，`latched_sender` 必须是你的 `ip:port`，`cmd_pkts_last_s` 是你的速率，`cmd_drop_allow` 必须保持为 0。

消费端：读文件。

```python
from record.state_reader import make_state_fn
state = make_state_fn()          # dict with ee_pose (16,), ee_ok; add with_wrench/with_joints
```

或者停掉 mirror unit，自己绑定 50002；这个端口只能有一个进程占用，推荐由 mirror 占用。

扩展主机（仅管理员）：新动词就是 `franka-ctl` 里的一个新 `case`；新 servo 模式就是 `franka-servo-run` 里的一个新 systemd 实例名，加上对应的 sudoers 行。流 A/B 的任何改动都按契约把 `FRST1` 升到 `FRST2`。

## 8. 可能出什么问题，以及它的表现

| 你看到 | 含义 | 处理 |
|---|---|---|
| `franka-remote` 退出码 255 | 没有 ssh：网线、IP、密钥未授权、主机没开 | `ping 10.10.0.2`，`nmcli con show franka-link`，向管理员确认你的密钥 |
| `franka-remote start pose` 退出码 1，journal 显示 precheck FAIL | 主机门禁：温度、RT 调优、镜像、机器人被占用 | `franka-remote status`；管理员在主机上跑 `franka-ctl preflight` |
| 退出码 75 | 机器人锁被占用，或已有 servo 活动 | `franka-remote status`；先停掉正在运行的 |
| bridge：“init pose timeout” | mirror 没收到数据：unit 没起、链路断、servo 往别处发布 | `franka_state_mirror --check`，`systemctl --user status franka-state-mirror` |
| 你在动，机械臂却 hold 着 | 你的数据包没被接受：`cmd_drop_allow`（源 IP 错误）、`cmd_drop_latch`（另一个发送端被 latch）、`cmd_pkts_last_s`=0（bridge 挂了） | 读 `/tmp/franka_link.txt` |
| 碰撞后机械臂冻结约 1.5 s | F/T freeze（35 N / 20 Nm） | 正常现象；放轻 |
| `reflex_count` 增加，约 2 s 后机械臂重新进入控制 | 机器人 reflex（碰撞、关节限位） | 正常现象；反复出现就停下检查工作空间 |
| `restart` 失败，提示“home sequence failed” | 回 home 运动期间发生 reflex（障碍物） | 清理工作空间，`franka-remote goto home`，`franka-teleop live` |
| preflight：时钟偏差 WARN/FAIL | NTP 漂移 | 修好你 PC 上的 NTP；录制数据只有在 10 ms 以内才可信 |
| 第二个发送端以 75 退出 | 发送端锁被占用 | 停掉另一个 |

## 9. 你可以预期的数字

来自 2026-09-08 的验收运行（[`ACCEPTANCE.zh-CN.md`](ACCEPTANCE.zh-CN.md)）：

| 量 | 值 |
|---|---|
| 控制平面 RTT（直连网线） | 平均 0.32 ms，最大 0.5 ms |
| FCI 链路（主机到机器人） | 1 kHz 下 10 000 次 ping，0 丢包，最大 0.48 ms |
| `communication_test` 成功率 | 1.00 / 1.00 / 1.00（三次运行） |
| 5 分钟 hold，无发送端 | 漂移 0.10 mm，0 次 missed cycle，0 次 reflex |
| 冷启动 `franka-teleop live` | 约 7 s 到“state ready” |
| `franka-teleop restart` | 约 8 s |
| 经 shim 的夹爪命令 | 往返约 2.5 s |
| 慢速圆轨迹跟踪（半径 3 cm，9 mm/s） | 平均误差 8 mm：来自阻抗柔顺，不是延迟（命令年龄 5 ms） |
| 链路中断 10 s | servo 不受影响，链路恢复后 mirror 在 < 0.2 s 内恢复 |
| teleop 会话（Quest，约 90 Hz） | 0 次 missed cycle；接触时一次 F/T freeze 和一次 reflex，均自行恢复 |

## 10. 软件栈与库

版本是验收运行所用的版本；固定版本写在 `client/requirements-*.txt` 和 `rt-host/docker/` 里。

### 实时主机

| 库 | 版本 | 角色 | 位置 |
|---|---|---|---|
| libfranka | 0.17.0（commit `4448c390`，`libfranka-common` `cd38d0ec`） | FCI 客户端：1 kHz `robot.control()` 力矩循环，`Model` 提供雅可比/科里奥利项，`Gripper` 走 TCP 1338，`automaticErrorRecovery` | 在 Docker 镜像内从源码构建；servo、`gripper_cmd`、`echo_robot_state`、`fk_probe` 链接它 |
| pinocchio | 3.9.0（来自 2026-04-13 的 ROS 2 Jazzy 快照的 `ros-jazzy-pinocchio`） | libfranka 模型所用的刚体运动学与动力学 | Docker 镜像；固定快照是因为在线仓库已经升到 pinocchio 4.0 |
| Eigen | 3.4 | 控制器中的矩阵和四元数 | Docker 镜像 |
| Poco | 1.11 | libfranka 的网络层 | Docker 镜像 |
| fmt | 9.1 | libfranka 内部的日志 | Docker 镜像 |
| Docker 镜像 `franka-rt:0.17.0-jazzy` | Ubuntu 24.04 + 上述各项，gcc 13 | 与主机 OS 无关的、可复现的 servo 用户空间；用 `--privileged --network host --ulimit rtprio=99 --ulimit memlock=-1` 运行 | `rt-host/docker/Dockerfile`、`build-image.sh`、`build-inside.sh` |
| franky（`franky-control`） | 1.1.3，针对 libfranka 0.17.0 构建的 wheel，主机上的 Python 3.10 venv | 底层用 libfranka 的点到点运动：`franka-ctl goto home` 运行 `franky_tools/goto_home.py`，它先抬离桌面，再用正运动学检查关节路径，然后才动；另有负载标定和夹爪工具 | `rt-host/franky_tools/`，通过 `rt-host/bin/franka-py` 运行；仅在没有 servo 活动时 |
| systemd、nftables、NetworkManager、DKMS `r8126` | Ubuntu 22.04 自带 | 服务监管、端口防火墙、两个网卡配置、机器人网卡驱动 | `rt-host/host/` |

servo 本身（`teleop/cartesian_pose_servo.cpp` 加 header-only 的 `teleop/rt/`）没有其他依赖：UDP、三重缓冲和实时设置代码都是纯 POSIX。

### 客户端

| 库 | 版本 | 角色 | 位置 |
|---|---|---|---|
| `teleop`（Spes Robotics） | 0.1.5 | 以 HTTPS 提供 WebXR 页面，并以头显帧率把手柄位姿变成 Python 回调；`client/teleop/frontend_swapped/` 里替换过的前端覆盖了它的 UI，提供 Trigger/Grip 按键映射 | `teleop/02_webxr_to_franka.py` |
| FastAPI + uvicorn | 0.136.1 / 0.46.0 | `teleop` 底下的 web 服务器（端口 4443，WebSocket `/ws`） | 由 `teleop` 拉入 |
| numpy、scipy、transforms3d | 2.4.4 / 1.17.1 / 0.4.2 | 4x4 位姿代数、旋转缩放（`scipy.spatial.transform.Rotation`）、四元数转换 | bridge |
| 仅 Python 标准库 | 3.x | 状态 mirror、发送端锁、FRST1 测试生成器、preflight 辅助脚本 | `client/bin/franka_state_mirror`、`client/teleop/franka_sender_lock.py`、`client/tests/fake_frst1.py` |
| pyrealsense2 | 2.57.7 | 带硬件时间戳的 Intel RealSense 采集，曝光锁定 | `client/record/cameras/grabber.py` |
| h5py、opencv-python、PyYAML | 3.16.0 / 4.13.0 / 6.0.3 | HDF5 episode 文件、图像处理、`cameras.yaml` | `client/record/` |
| OpenSSH、NetworkManager、systemd 用户 unit | 自带 | 控制通道、私有链路配置、mirror 服务 | `client/install.sh` |

libfranka 固定在 0.17.0，是因为机器人的系统镜像（5.7.2）说的是 research-interface 第 9 版，只有 libfranka 0.15 到 0.17 实现了它；单独升级任何一侧都会断开连接。慢速运动交给 franky，快速运动交给 C++ servo：franky 的 Python API 让点到点运动周围的安全检查容易编写，而 1 kHz 阻抗循环需要编译型控制器。
