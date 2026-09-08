[English](README.md) | 中文

# frlink

一套面向 Franka FR3（带 Franka Hand）的双机方案。一台精简、版本固定的实时主机（RT host）独占机器人，在隔离 CPU 上的容器里运行 1 kHz 笛卡尔阻抗 servo。任意一台客户端 PC 通过一根直连网线接到它。跨过这根网线的只有三条窄通道：入向的 128 字节 UDP 目标位姿流，出向的纯文本 UDP 状态流，以及经 ssh forced command 传递的少数几个控制动词。客户端从不链接 libfranka，也看不到机器人的网络，所以可以随意替换、升级或重启，不必碰机器人那一侧。本仓库同时包含两半，以及它们之间冻结的接口。文档化的用途是从 WebXR 头显做 6 自由度 teleop，并带夹爪控制。

```
 客户端 PC（你的）                                  RT host "alienware"                  FR3
 =================                                 ===================                  ===
 02_webxr_to_franka.py ---- UDP 128 B O_T_EE ---> :50001  cartesian_pose_servo -------FCI-----> 172.16.0.2
 (franka-teleop)                                         docker franka-rt, CPU 2, FIFO 99      1 kHz
                                                          |
 /tmp/franka_*.txt <-- 状态镜像 <------ UDP FRST1 <-------+ 发布线程（CPU 4-15）
                       (:50002)
 franka-remote <verb> ---- ssh forced command ----> franka-ctl  -> systemd franka-servo@pose
                                                                -> goto home / gripper / echo / comm-test
 NIC 10.10.0.1/24 <=========== 直连网线 ================> frlink0 10.10.0.2/24
                                                          nftables：仅 10.10.0.0/24，仅 ssh + udp 50001
```

## 下一步去哪

| 你想要 | 读 |
|---|---|
| 从你自己的 PC 驱动机器人：原理、代码讲解、会话流程 | [`docs/GUIDE.zh-CN.md`](docs/GUIDE.zh-CN.md) |
| 在机器人自己的控制台里解锁、激活、手动引导或恢复机器人 | [`docs/DESK.zh-CN.md`](docs/DESK.zh-CN.md) |
| 线上格式、地址、速率和退出码（已冻结） | [`docs/INTERFACE.zh-CN.md`](docs/INTERFACE.zh-CN.md) |
| 主机 CPU 与线程布局以及安全包络，表格形式 | [`docs/ARCHITECTURE.zh-CN.md`](docs/ARCHITECTURE.zh-CN.md) |
| 验收流程与实测数字 | [`docs/ACCEPTANCE.zh-CN.md`](docs/ACCEPTANCE.zh-CN.md) |
| 安装客户端这一半 | [`client/README.zh-CN.md`](client/README.zh-CN.md) |
| 配置或管理主机，授权密钥 | [`rt-host/README.zh-CN.md`](rt-host/README.zh-CN.md) |

## 连接你的 PC

1. 用一根直通网线把你的 PC 接到主机的 `frlink0` 口（扩展坞上的 ASIX USB 网口，不是内置 RJ45，后者接机器人）。不经过交换机。
2. 给你的网卡设 `10.10.0.1/24`，不设网关。主机是 `10.10.0.2`；`ping` 它。这个地址是固定的：servo 只向一个客户端发布状态，只从一个源接受命令，两者都是 `10.10.0.1`（见 [`rt-host/README.zh-CN.md`](rt-host/README.zh-CN.md#同一时间只有一个客户端来访-pc-必须是-101001)）。
3. 生成一对 ed25519 密钥。把公钥交给主机所有者，由他加到主机的 `authorized_keys` 里，并带上 `franka-ctl` forced command 前缀（格式见 `rt-host/README.zh-CN.md`）。测试：`ssh -i <key> rongxuan_zhou@10.10.0.2 status`。
4. 按 [`client/README.zh-CN.md`](client/README.zh-CN.md) 安装客户端这一半，然后按 [`docs/GUIDE.zh-CN.md`](docs/GUIDE.zh-CN.md) 第 5 节运行一次会话。在这之前的 Desk 步骤（解锁关节、Activate FCI、按下 Enable）在 [`docs/DESK.zh-CN.md`](docs/DESK.zh-CN.md)。

## 安全

任何可能让机械臂动起来的步骤，都要把**急停（user stop）握在手里**。`comm-test`、`goto home` 和 `restart pose home` 会自行移动机械臂；运行中的 servo 会把它移到发送端指向的任何地方。执行 `goto home`、`comm-test` 以及启动 servo 之前，先清空工作空间。

**servo 本身没有工作空间防护。** `cartesian_pose_servo` 只是跟随给它的目标。它仅有的限制是：命令停止后 200 ms 的 hold，外力超过 35 N / 20 Nm 时 1.5 s 的 freeze，每关节力矩上限，libfranka 的速率限制，以及机器人自身的碰撞 reflex（30 N / 30 Nm）。客户端上的 bridge 负责工作空间盒、每帧 0.0055 m 的步长上限（90 Hz 下 0.5 m/s）、0.5 的旋转缩放和 dead-man 扳机；数值列在 [`docs/ARCHITECTURE.zh-CN.md`](docs/ARCHITECTURE.zh-CN.md)。你自己写的任何发送端都必须实现等价的防护；主机不会替你做。

**servo 活动期间，绝不要运行 `echo`、`goto`、`comm-test` 或任何直连 FCI 的工具。** `franka-ctl` 会拒绝它们（退出码 75）。在主机上手动运行的工具绕过了这个检查，而第二个 libfranka 连接会终止控制循环。

同一时间只有一个发送端。servo latch 第一个发送端并丢弃其余的；客户端软件包持有一把本地锁，第二个本地程序会明确报错退出。两者都不要绕过。

会话期间不要在实时主机上跑 GPU 和 CPU 任务、浏览器和桌面应用；热降频会表现为 missed cycle。

## 仓库布局

```
README.md                 本文件
CONTRIBUTING.md           改动约定
docs/
  GUIDE.md                从你自己的 PC 使用机器人：原理、代码、会话流程
  DESK.md                 机器人自己的控制台：状态、指示灯、手动引导、错误恢复
  INTERFACE.md            冻结的双机契约（地址、格式、动词、退出码）
  ARCHITECTURE.md         双机设计、三条数据流、主机线程/CPU 布局、安全包络
  ACCEPTANCE.md           day-1 / day-2 流程与 2026-09-08 的 day-1 数字
rt-host/                  实时主机这一半（主机上 ~/franka 的导出）
  README.md               主机做什么、配置顺序、franka-ctl 动词、密钥授权
  bin/                    franka-ctl、franka-preflight、franka-run、servo unit 辅助脚本
  host/                   provision-rt.sh、verify-rt-host.sh、franka-rt-tune.sh、etc/ 树
  docker/                 franka-rt:0.17.0-jazzy 的 Dockerfile + 构建脚本
  teleop/                 servo 与运动辅助程序源码、rt/ 头文件单元、tests/
  tests/                  基准测试、FRST1 sink、假发送端、机器人日检查清单
  franky_tools/           基于 franky 的主机原生脚本：goto_home.py（`goto home` 用）、标定、wiggle（无 servo 活动时）
  docs/                   主机上保存的 INTERFACE、RUNBOOK、ASSESSMENT
client/                   客户端这一半（状态 mirror、franka-remote、preflight、bridge、franka-teleop），见 client/README.md
```

## 许可证

MIT，见 [`LICENSE`](LICENSE)。libfranka（Apache-2.0）和其他依赖各自保留自己的许可证。
