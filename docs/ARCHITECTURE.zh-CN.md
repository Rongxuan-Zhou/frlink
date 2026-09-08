[English](ARCHITECTURE.md) | 中文

# 架构参考

拆分的理由、代码走读和会话流程写在 [`GUIDE.zh-CN.md`](GUIDE.zh-CN.md) 第 2 到 4 节；线上格式冻结在
[`INTERFACE.zh-CN.md`](INTERFACE.zh-CN.md)。本页记录那两份文档没有展开的内容：主机内核与线程布局、
以表格形式列出的安全包络，以及控制通道的细节。

## 为什么用两台主机

力矩控制的 FR3 需要一个从不漏拍的 1 kHz 控制环；而遥操作或策略工作站需要 GPU、浏览器、相机和较新的
内核。在之前的单机配置上，这两者放在一起会让每个核心都出现 6 ms 的停顿，一次内核更新还弄坏了网卡驱动。
所以由一台小型、版本钉死的 RT host（实时主机）独占机器人，可互换的客户端 PC 通过一根直连网线、经由一个
冻结的最小接口与它通信。抖动数据见 GUIDE.zh-CN.md 第 2.1 节。

```
 client PC (10.10.0.1)                              RT host (10.10.0.2)                 FR3
 --------------------                               -------------------                 ---
 02_webxr_to_franka.py ----- UDP 50001 ----------> cartesian_pose_servo -------------FCI-> 172.16.0.2
 /tmp/franka_*.txt <-- state mirror <-- UDP 50002 <-- FRST1 publisher (same process)  1 kHz
 franka-remote <verb> ---- ssh forced command ----> franka-ctl -> systemd / goto / gripper
 NIC 10.10.0.1/24 <======= direct cable ========> frlink0 10.10.0.2/24     enp110s0 172.16.0.1/24
```

## 三条数据流

三条流都走同一根网线。其中任何一条的改动都会升级协议标签（`FRST1` -> `FRST2`）。格式、消息名和速率见
GUIDE.zh-CN.md 第 2.2 节。

| 流 | 方向 | 传输 | 格式 |
|---|---|---|---|
| 命令（command） | 客户端 -> 主机 | UDP `10.10.0.2:50001` | 128 字节 = 16 个小端 float64 = 列主序 4x4 `O_T_EE`（平移量在索引 12、13、14） |
| 状态（state） | 主机 -> 客户端 | UDP `10.10.0.1:50002` | `FRST1 <seq> <epoch_ns> <t_real_ns> <t_mono_ns> <name>\n<body>` |
| 控制（control） | 客户端 -> 主机 | ssh forced command | `status`、`preflight`、`start pose`、`stop`、`restart pose [home]`、`goto home`、`gripper ...`、`echo`、`comm-test --yes` |

命令格式在拆分之前就已存在，保留它是为了让现有发送端不用改动就能工作；其它任何长度的数据包都会被丢弃
并计数（`cmd_drop_size`）。阻抗律（K_t 1000 N/m，K_r 80 Nm/rad，接触时都软化到 25 %）使用 Franka Hand
负载模型（0.25 kg，COM z 0.05 m）。这些都是编译期默认值；`franka-servo@pose` 不传任何任务参数。

在状态流中，`body` 与 servo 写入主机上 `/tmp/franka/<name>` 的内容逐字节相同，`epoch_ns` 是 servo 进程
启动时的 `CLOCK_REALTIME`。

控制动词通过一条形如
`command="/home/rongxuan_zhou/franka/bin/franka-ctl",no-pty,...,from="10.10.0.0/24"` 的 `authorized_keys`
条目到达；sshd 把动词放在 `SSH_ORIGINAL_COMMAND` 中传入。令牌校验、退出码和机器人锁见
GUIDE.zh-CN.md 第 3.1 节。

## 主机内核、CPU 与线程布局

内核命令行：`preempt=full isolcpus=domain,managed_irq,2,3 nohz_full=2,3 rcu_nocbs=2,3
irqaffinity=0-1,4-31 threadirqs`。CPU 2 和 3 从调度器的域和默认 IRQ 掩码中移除。`franka-rt-tune` 把它们
设为 performance 调频策略，禁用比 C1 更深的 C-state，并把每个 `enp110s0-*` MSI-X 向量钉到 CPU 3，其 IRQ
线程运行在 `SCHED_FIFO 85`。

servo 运行在 `franka-rt:0.17.0-jazzy` 容器内（`--privileged --network host
--ulimit rtprio=99 --ulimit memlock=-1`），由 `franka-servo@pose` 单元在 `systemd-inhibit` 下启动
（运行期间不休眠，合盖也不触发动作）。

| 线程 | CPU | 策略 | 职责 |
|---|---|---|---|
| `main`（libfranka `robot.control()`） | 2 | `SCHED_FIFO 99`（由 libfranka 设置） | 1 kHz 力矩回调：从三重缓冲读取目标，计算笛卡尔阻抗力矩，把当前 EE / 力旋量 / 关节状态发布到三重缓冲 |
| `udp_cmd` | 4-15 | `SCHED_OTHER` | 带 100 ms 超时的 `recvfrom` 循环：白名单、长度、latch；写入 `target_raw` |
| `ee_writer`、`wrench_writer`、`joint_writer` | 4-15 | `SCHED_OTHER` | 周期性重写文件 + 发送 FRST1，两者共用同一个格式化函数 |
| `init_resend` | 4-15 | `SCHED_OTHER` | 每 1 s 重发一次 `franka_init_pose.txt` |
| `link_writer` | 4-15 | `SCHED_OTHER` | 以 5 Hz 写 `franka_link.txt` |

`main()` 中的顺序：`mlockall(MCL_CURRENT|MCL_FUTURE)`，预触发 8 MB 栈，设置辅助 CPU 掩码，派生辅助线程
（它们继承该掩码），把 `main` 钉到 CPU 2，然后把控制权交给 libfranka。与 tick 共享的唯一状态是无锁的
单生产者/单消费者三重缓冲。

`missed_cycles_total` 统计超过 1.5 ms 的回调间隔；验收门限是一个会话内至多 20 次。健康主机上的
`tick_max_us_1s` 在 1100-1140 us 附近（1 kHz 周期加抖动）。

控制面网卡（`frlink0`，一个 USB 千兆口）不是实时的：它只承载 90 Hz 的命令流、20 Hz 的状态流和 ssh。
机器人网卡（`enp110s0`，RTL8126，驱动 `r8126`）是唯一的硬实时路径，nftables 会丢掉它上面除与 `172.16.0.2`
收发之外的一切流量。

## 安全包络

这些常量位于 servo 源码中，已核实与拆分前的 servo 完全一致。

| 机制 | 数值 | 行为 |
|---|---|---|
| 命令保持 | 200 ms 内没有被接受的数据包 | target := 当前 `O_T_EE`；力矩控制继续（手臂保持柔顺，不会掉落，也不会漂移）。下一个被接受的数据包即解除。 |
| F/T 紧急冻结 | `O_F_ext_hat_K` 上 \|F\| > 35 N 或 \|T\| > 20 Nm | 目标冻结 1500 ms，被接受的数据包计数但丢弃（`franka_link.txt` 中 `freeze=1`），然后释放 |
| 力矩上限 | 每关节 0.85 x [87, 87, 87, 87, 12, 12, 12] Nm，随后 `franka::limitRate` | 命令离开回调之前的硬钳位 |
| Reflex 重试 | 最多 50 次自动恢复 | 遇到 libfranka reflex（碰撞、关节限位）时，servo 调用 `automaticErrorRecovery` 并重新进入 `robot.control()`；两者之间 `recovering=1`，`reflex_count` 递增。超过 50 次进程退出，单元停止。 |
| 机器人碰撞阈值 | 30 N / 30 Nm（servo 启动时设置） | 低于软件冻结阈值，所以真实碰撞通常先表现为 reflex |
| 发送端白名单 | `--cmd-allow`（servo）和 nftables 的 `cmd_allow` 集合 | 来自其它任何源 IP 的数据报都到不了 servo（`cmd_drop_allow`） |
| 发送端 latch | 第一个被接受的 `(ip, port)`；静默 1 s 后释放 | 第一个发送端存活期间，第二个发送端无法注入目标（`cmd_drop_latch`）；交接最多需要 1 s 加上保持时间 |
| Servo 预检 | 单元的 `ExecStartPre` | 要求 `rt-tune.ok` 存在、nftables 已加载、接交流电、封装温度 < 85 C、没有其它 servo/goto/echo 容器、绑定地址已配置、机器人链路已起来、镜像和二进制齐全，否则拒绝启动 |
| 互斥 | `flock /run/lock/franka-robot.lock`；servo 活动期间拒绝 `goto`/`echo`/`comm-test` | 同一时刻只有一个 libfranka 连接 |
| 温度守护 | 1 Hz 定时器，`/run/franka/temp_c` | `status` 和预检读取它；RUNBOOK 的故障表列出了接近 100 C 时观察到的停顿症状 |

`cartesian_pose_servo` 没有工作空间守护、没有包围盒，除机器人自身强制的以外也没有关节限位检查；给它
什么目标它就跟什么目标。工作空间安全在上一跳、也就是客户端的桥接程序（`02_webxr_to_franka.py`）里
强制执行：

| 守护 | 数值 | 效果 |
|---|---|---|
| 工作空间盒（基座坐标系） | x [0.150, 0.700] m，y [-0.550, 0.500] m，z [0.100, 0.656] m | 目标平移量在三个轴上都被硬裁剪；控制器离开盒子时锚点会被重置，这样操作者不会带着一个偏移量回到盒内 |
| 步长上限 | 90 Hz 下每帧 0.0055 m = 0.5 m/s | 把笛卡尔速度限制在 ISO/TS 15066 协作限值；0.5 m/s 折合关节约 0.7 rad/s，是 FR3 关节速度限值的 35 % |
| 旋转缩放 | 0.5 | 控制器手腕角度映射到机器人上的一半角度；消除手腕抖动放大 |
| 死人开关 | 按住 Trigger | 只有按住扳机时桥接程序才发送目标；松开即停止数据流，servo 在 200 ms 内进入保持 |
| 夹爪 | Grip 键切换开 / 合 | 经主机上的 `gripper_cmd`（`franka-ctl gripper`），它可以与 servo 并行运行 |

其它发送端（策略、脚本）必须自己实现同样的盒子和步长上限。操作者手中的 user stop（急停）按钮是最后
一道防线。

## 客户端

状态 mirror 校验什么、如何写文件，见 GUIDE.zh-CN.md 第 4.1 节。有两个细节关系到设计。mirror 状态行里的
`skew_ms`（接收时的墙钟时间减去数据报的 `t_real_ns`）是整个系统里唯一的时钟比较。客户端 preflight 会对
超过 10 ms 的偏差报警，因为否则客户端拿 mirror 状态做时间戳的一切（日志、录制）都会悄悄偏掉。而
`/tmp/franka_sender.lock` 上的本地 `flock` 存在的目的，是让第二个本地发送端大声失败，而不是被 servo latch
静默丢弃。
