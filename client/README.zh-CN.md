[English](README.md) | 中文

# client：通过 RT host / 实时主机遥操作并录制 Franka FR3

一台来访的 Linux PC 要使用 [`../rt-host/`](../rt-host/README.zh-CN.md) 后面的 FR3 所需的全部内容。这台 PC 从不直接与机器人通信：目标位姿通过 UDP 发出，机器人状态通过 UDP 返回，控制动词则通过一把仅限 `franka-ctl` 的 ssh 密钥执行。接口契约：[`../docs/INTERFACE.zh-CN.md`](../docs/INTERFACE.zh-CN.md)。为什么这样设计，以及 `/tmp/franka_*.txt` 文件里有什么：[`../docs/GUIDE.zh-CN.md`](../docs/GUIDE.zh-CN.md)。

```
 your PC (10.10.0.1)                                            RT host (10.10.0.2)
 -------------------                                            -------------------
 bin/franka-teleop -> teleop/02_webxr_to_franka.py --UDP 50001--> cartesian_pose_servo (1 kHz)
 /tmp/franka_*.txt  <-- bin/franka_state_mirror  <--UDP 50002-- FRST1 state publisher
 record/record_episodes.py (cameras + /tmp/franka_*.txt -> HDF5)
 bin/franka-remote <verb> ------- ssh forced command ---------> franka-ctl (start/stop/goto/gripper)
 NIC $FRANKA_LINK_IFACE <============ direct cable ===========> frlink0
```

你需要一台带空闲以太网口、NetworkManager、systemd 用户会话和 Python 3 的 Linux PC，以及一台已上电且 `frlink0` 口空闲的实时主机（同一时间只允许一个客户端，地址必须正好是 `10.10.0.1`）。遥操作需要一台 Meta Quest，录制需要 Intel RealSense 相机。主机管理员会把你的公钥追加到主机的 `authorized_keys`。

## 快速开始

1. 接线与地址。把你的网卡插到主机的客户端口，填写 `config.env`：
   ```
   cp config.env.example config.env
   $EDITOR config.env        # FRANKA_CTL_USER, FRANKA_LINK_IFACE (ip -br link), FRANKA_PY
   ```
   创建链路 profile（`install.sh` 会再打印一遍）：
   ```
   nmcli con add type ethernet ifname <iface> con-name franka-link ipv4.method manual \
       ipv4.addresses 10.10.0.1/24 ipv4.never-default yes ipv6.method disabled connection.autoconnect yes
   nmcli con up franka-link && ping -c 3 10.10.0.2
   ```
2. 安装。`./install.sh` 把 `bin/` 软链到 `~/.local/bin`，生成 `~/.ssh/franka_ctl` 和 `~/.ssh/config` 里的 `Host alienware-rt` 块，启动 `franka-state-mirror` 用户单元，并打印交给主机管理员的 `authorized_keys` 行。每次改动 `config.env` 之后都要重新运行它。
3. Python。往 `FRANKA_PY` 指定的解释器里 `pip install -r requirements-teleop.txt`；录制则 `pip install -r requirements-record.txt`。mirror 使用系统的 `python3`。
4. Preflight。`franka-client-preflight` 必须以 `CLIENT PREFLIGHT PASS` 结束（链路、ssh、mirror、时钟偏差、没有竞争发送者）。每次会话开始时都运行一遍。
5. 机器人上电。在 Desk（`https://172.16.0.2/desk/`，仅主机可访问）上：解锁关节，**Activate FCI**，按下使能按钮。只有主机管理员能做这一步。
6. 回 home，然后遥操作。
   ```
   franka-remote goto home          # arm moves to the factory-ready pose (~10 s)
   franka-teleop live               # start the servo on the host, wait for state, start the bridge
   franka-teleop status             # bridge PID, mirror health, host units
   ```
   在 Quest 上打开 `https://<your PC>:4443/`。**站到机器人正前方、面朝机器人，再按 Enter VR。** Trigger 移动机械臂，Grip 切换夹爪。**先在网页里按 Exit VR，再**执行 `franka-teleop stop`。操作流程、手柄映射和限制：[`teleop/README.zh-CN.md`](teleop/README.zh-CN.md)。
7. 录制。teleop 运行期间，在第二个终端里：
   ```
   python3 record/state_reader.py --selftest --secs 3        # ee_ok 60/60
   python3 record/record_episodes.py --out-dir ~/datasets/<task> --prefix <task>
   ```
   按键：`s` / `e` / `d` / `q`。数据集布局：[`record/README.zh-CN.md`](record/README.zh-CN.md)。
8. 停止。`franka-teleop stop`（bridge + servo）。`franka-teleop restart` = 停止、回 home、重新启动（实测约 8 s，机械臂会动）。用 `q` 退出录制器，绝不要 `kill -9`；最后一段 episode 还在写入。

## 命令

| 命令 | 用途 |
|---|---|
| `franka-client-preflight` | PASS/WARN/FAIL 检查清单；任一 FAIL 则以 exit 1 退出 |
| `franka-remote <verb>` | `status`、`preflight`、`start pose`、`stop`、`restart pose [home]`、`goto home`、`gripper open\|close\|width w\|grasp w\|homing\|read`、`echo`、`comm-test --yes`。退出码透传：64 被拒绝，75 忙，124 本地超时，255 ssh/链路 |
| `franka-teleop home\|servo-only\|bridge-only\|live\|restart\|stop\|status\|wait-ready [s]` | 会话编排器；参数在脚本和 `config.env` 里 |
| `franka_state_mirror --check` | mirror 健康状态（守护进程存活、EE 文件新鲜、16 个 token）；守护进程作为用户单元运行 |
| `bin/gripper_cmd`、`bin/goto_home`、`bin/echo_robot_state` | shim 别名（`franka-fci-shim`），转发到 `franka-ctl`；bridge 保持原有的调用形式 |
| `tests/fake_frst1.py --dst 127.0.0.1:50002` | 本地 FRST1 生成器，不需要主机就能测 mirror |
| `record/selftest.py [secs]` | 只测相机，HDF5 + 时间戳检查 |

日志：`/tmp/franka_live.log`（bridge）、`/tmp/franka_servo.log`（主机动词）、`journalctl --user -u franka-state-mirror`、`/tmp/franka_mirror.txt`（1 Hz 状态行：`epoch seq src age_ms skew_ms rx drop_*`）。

mirror 在 `/tmp` 下的文件：`franka_init_pose.txt`（16 个值，每个 servo 实例写一次）、`franka_current_ee.txt`（16 个值，20 Hz）、`franka_wrench.txt`（6 个值，20 Hz）、`franka_joint_state.txt`（28 个值，10 Hz）、`franka_link.txt`（13 个 `key=value`，5 Hz）。servo 或链路静默时它们停止更新；不会删除任何文件。消费者自行应用过期规则（250 ms）。

## 故障表

| 症状 | 检查 | 处理 |
|---|---|---|
| `franka-remote` 以 255 退出 | `ping 10.10.0.2`；`nmcli con show franka-link`；密钥在 `~/.ssh/franka_ctl`；主机密钥在 `known_hosts` 里 | 检查线缆 / `nmcli con up franka-link`；问管理员密钥是否已授权；`ssh-keyscan -t ed25519 10.10.0.2 >> ~/.ssh/known_hosts` |
| `franka-remote` 以 64 退出 | 动词不在允许集合内，或带了多余 token | 只能用上面列出的动词；不能带 shell 语法 |
| `franka-remote` 以 75 退出 | `franka-remote status` | 主机忙（另一个动词在运行，或有 servo 活动）；等待或 `stop` |
| bridge 报 init pose timeout，`/tmp/franka_current_ee.txt` 过期 | `franka_state_mirror --check`；`systemctl --user status franka-state-mirror`；`franka-remote status`（servo 活动吗？主机上的 `FRANKA_STATE_DST` = 你的 IP 吗？） | 重启该单元；主机的 `/etc/franka/servo.env` 必须把 `FRANKA_STATE_DST` 指向 `10.10.0.1:50002` |
| 遥操作时机械臂保持不动；`franka_link.txt` 的 `cmd_age_ms` > 200 | `cmd_drop_allow` 上升 -> 你的 IP 不在允许列表；`cmd_drop_latch` 上升 -> 另一个发送者持有 latch；`cmd_pkts_last_s`=0 -> bridge 已死 | 修好发送者；这个 hold 是安全的 |
| 第二个 bridge/录制器以 75 退出 | `cat /tmp/franka_sender.lock` | 先停掉持有者 |
| preflight 时钟偏差 WARN/FAIL | 在两台机器上 `timedatectl` | 两台都必须 NTP 同步；> 10 ms 会劣化录制的动作标签 |
| bridge 以 2 退出，"EE not inside bbox" | 机械臂在工作空间盒之外 | `franka-remote goto home`（或 `franka-teleop restart`），或者放宽 `FRANKA_WS_*` |
| Quest 打开 `https://<PC>:4443` 什么都没有 | 防火墙、是否同一网络、证书警告未接受 | 放行 TCP 4443；接受自签名证书；或者用 Tailscale Funnel（`FRANKA_TELEOP_URL`） |
| preflight 发现游离的 `fake_frst1`/`state_sink`/第二个 mirror | `ps`、`ss -lunp \| grep 5000` | 杀掉它 |
| `pkill -f <pattern>` 杀掉了你自己的 shell | 模式匹配到了 `bash -c` 命令行 | 给模式加锚点；这里的脚本正因如此使用 `ps \| grep -v` |
| 主机上 servo 退出（reflex，`TCP interrupted`） | `franka-remote status`；Desk 显示错误 | 在 Desk 上清除错误，重新激活 FCI，`franka-teleop servo-only`（bridge 继续运行）或 `franka-teleop restart` |

## 目录布局

```
client/
  README.md                    this file
  config.env.example           copy to config.env (local, not committed)
  install.sh                   idempotent client setup
  requirements-teleop.txt      bridge environment (FRANKA_PY)
  requirements-record.txt      recorder environment
  bin/                         franka_state_mirror, franka-remote, franka-fci-shim (+ aliases),
                               franka-client-preflight, franka-teleop
  systemd/                     franka-state-mirror.service template (@ROOT@ substituted by install.sh)
  teleop/                      02_webxr_to_franka.py (bridge), 01_webxr_pose_reader.py (link test),
                               franka_sender_lock.py, frontend_swapped/ (Quest page), README.md
  record/                      record_episodes.py, selftest.py, state_reader.py, cameras/, config/cameras.yaml, README.md
  tests/fake_frst1.py          FRST1 generator for the mirror
```

每个脚本都从自身位置推导仓库根目录，所以这个目录可以放在任何地方；只有 `config.env` 是与机器相关的。
