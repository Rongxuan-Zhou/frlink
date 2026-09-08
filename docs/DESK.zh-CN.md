[English](DESK.md) | 中文

# Franka Desk：机器人自己的控制台

Desk 是内置在 FR3 控制柜里的网页界面，也是唯一能解锁、激活机器人、进入手动引导模式或从错误中恢复的地方。客户端工具默认有人已经先做完了这些步骤。实验室的机器人运行系统镜像 5.7.2，下面的菜单名称来自该版本；如果你的 Desk 看起来不一样，状态机是一样的。

## 1. 访问 Desk

控制柜在机器人网络上以 `https://172.16.0.2/desk/` 提供 Desk，这个网络止于实时主机（RT host）的内置网卡，不会路由到客户端链路。因此：

- 在实时主机上：用浏览器打开该 URL（自签名证书；接受一次即可）；
- 从另一台在主机上有普通 ssh 账号的机器：`ssh -L 8443:172.16.0.2:443 <user>@10.10.0.2`，然后打开 `https://localhost:8443/desk/`；
- 只有 forced-command 密钥的客户端无法访问 Desk。请找主机管理员。

用机器人的管理员账号登录。Desk 同一时间只把控制权交给一个浏览器会话；第二个会话以只读方式打开，带一个“request control”按钮，第一个会话可以同意或拒绝。Desk 不响应你的点击时，看看顶栏有没有只读横幅。FCI 使用同一个控制 token：在主机上执行 `curl -k https://172.16.0.2/admin/api/control-token` 会打印谁持有它，以及 FCI 是否激活。

## 2. 状态与指示灯

机械臂底座上的环形灯显示机器人状态。下面的颜色是我们在实验室 FR3 上看到的；两者不一致时，以 Desk 侧栏的状态文字为准：

| 灯 | 状态 | 含义 | 如何退出 |
|---|---|---|---|
| 黄色，脉动 | 启动中 | 控制柜正在启动（约 1-2 分钟） | 等它停止脉动 |
| 黄色 | 已锁定或未激活 | 启动后抱闸闭合，或急停被按下，或使能按钮没有按下 | 解锁关节（Desk 侧栏），松开急停，按下使能按钮 |
| 蓝色 | 就绪（“Idle”） | 抱闸打开，已激活，无错误；机器人接受 FCI 控制、Desk 应用和引导 | 每次会话都从这个状态开始 |
| 白色 | 引导中 | 手动引导激活（你正按着 Pilot 按钮） | 松开按钮 |
| 绿色 | 执行中 | 一个 Desk 应用或任务正在运行 | 等待或停止该应用 |
| 红色 | 错误 | reflex 或安全违规让机械臂停了；需要确认 | 见第 5 节 |

FCI 控制不改变灯色：servo 运行时灯保持蓝色。servo 运行时出现红色，说明 servo 自己的恢复已经在进行（5.3 节）。

两个物理输入和指示灯一样重要：

- 急停（user stop，带线的蘑菇头按钮）会断开抱闸的安全回路，让机器人变成黄色；旋转即可释放。**任何不是你自己规划的运动，都要把它握在手里。**
- 使能按钮（enabling device，外部激活盒上的“Enable”按钮）：启动或急停之后，只有按过它机器人才算“已激活”；再按一次则取消激活。Desk 侧栏在“Robot”旁边显示该状态。

## 3. 让机器人上线（每次上电）

1. 打开控制柜。等黄色脉动停止、Desk 加载完成。
2. 在 Desk 侧栏点 **Unlock joints**。抱闸逐个关节咔哒松开，状态从“locked”变为“Idle”。按钮是灰色说明急停被按下了。
3. 按下使能按钮。灯变蓝。
4. 要用客户端工具：右上角菜单（三条横线）-> **Activate FCI**。Desk 显示一条横幅；FCI 激活期间，Desk 应用和“move”控件被禁用，引导只在 servo 没有运行时可用。
5. 到这时客户端命令才能用：`franka-remote status` 显示 FCI 链路已连上，`franka-remote goto home` 会移动机械臂。

收尾：在客户端执行 `franka-teleop stop`，然后在 Desk 里，如果别人要用 Desk 应用就 **Deactivate FCI**，如果机械臂将被留在原地就 **Lock joints**，再从右上角菜单 **Shut down**，而不是直接按电源开关；等 LED 全部熄灭后再关开关。

## 4. 手动引导机械臂（“拖拽”）

手动引导是移动机械臂最快的方式：挪到新的起始位姿、挪开、或者在越限后拖回关节限位以内。

前提：灯为蓝色，急停已释放，没有 FCI 客户端在运行（活动的 servo 独占关节；先 `franka-teleop stop`）。单独激活 FCI 并不妨碍引导。

1. 在 Desk 顶栏把操作模式从 **Execution** 切到 **Programming**，机械臂会以降低的速度运行，并启用 Pilot 按钮。
2. 在侧栏控件里选引导模式：**Free**（全部六个自由度）、仅 **Translation**、仅 **Rotation**，或 **User**（你勾选的轴）。肘部运动可以单独锁定或释放。
3. 按住 Pilot 上的引导按钮（腕部的圆盘；在 FR3 上是握住圆盘时手指下方的那一对按钮）。灯变白，机械臂变得没有重量。松开就锁在原地；灯变回蓝色。
4. 读取位姿：`franka-remote echo` 以 JSON 打印 `O_T_EE` 和关节角（servo 必须已停止，如果你刚在引导，那它就是停止的）。存到你工作流保存位姿的地方。
5. 再次启动 servo 之前切回 **Execution**。

引导时一只手放在 Pilot 上，另一只手空着，不要把关节引导过它的限位。30 N / 30 Nm 的碰撞阈值是 servo 设置的，引导期间不生效。

## 5. 清除错误

### 5.1 Desk 显示什么

错误是侧栏里的红色横幅，带错误名称（例如 `cartesian_reflex`、`joint_position_limits_violation`、`communication_constraints_violation`）和一个 **Acknowledge**（或 **Recover**）按钮。它执行的自动恢复和 libfranka 暴露的 `automaticErrorRecovery` 相同：抱闸保持打开，控制器状态复位，灯回到蓝色。错误文本留在日志里（侧栏 -> Logs）。

### 5.2 常见的几种

| 错误 / 症状 | 原因 | 处理 |
|---|---|---|
| `cartesian_reflex`、`joint_reflex` | 机械臂撞到东西，或被推的力超过碰撞阈值 | 移除障碍物，在 Desk 里 Acknowledge（或让 servo 自行恢复，5.3），然后 `franka-remote goto home` |
| `joint_position_limits_violation`、`cartesian_position_limits_violation` | 某个关节或末端被驱动到了硬限位 | Acknowledge；如果机械臂仍在限位之外，先手动引导它回来（第 4 节）再启动任何东西 |
| `joint_velocity_violation`、`cartesian_velocity_violation` | 目标在一步内跳得太远 | Acknowledge；在客户端减小步长（`FRANKA_MAX_STEP`），或修好发出跳变的那个发送端 |
| `communication_constraints_violation` | FCI 客户端漏掉了太多 1 ms 周期 | servo 主机出现了停顿：检查 `franka_link.txt`（`missed_cycles_total`）、主机的热状态（`franka-remote status`），然后重启 servo |
| 灯黄色，Desk 显示“not activated” | 急停被按下，或使能按钮被切换了 | 释放急停，按下使能按钮 |
| 灯黄色，启动后显示“joints locked” | 抱闸闭合 | Unlock joints |
| Desk 只读 / 点击无响应 | 另一个浏览器持有控制权 | 在你的会话里 Request control，或关掉另一个 |
| “FCI already in use” / servo 启动时因连接错误退出 | 另一个 FCI 客户端（一个 `echo`、一个 `goto`、第二个 servo）占着通道 | 客户端上 `franka-remote status`，主机上 `franka-ctl status`；停掉它 |
| Desk 无法访问 | 控制柜关机、到主机的网线拔了、主机网卡 down | 在主机上：`ping 172.16.0.2`、`ip -br addr show enp110s0`；如果控制柜开着且链路正常，给控制柜断电重启 |

### 5.3 servo 自己会做什么

FCI 控制期间机械臂发生 reflex 时，`robot.control()` 在 servo 内部抛出异常。servo 不会退出：它冻结目标，调用 `automaticErrorRecovery`，等 500 ms，重新读取位姿，再过 2 s 重新进入控制，最多 50 次。在客户端上，`/tmp/franka_link.txt` 里 `reflex_count` 增加，`recovering=1` 短暂出现；机械臂在原地恢复。只有在恢复失败（灯保持红色，`franka-remote status` 显示 unit 处于 inactive）或限位违规需要把机械臂引导回来时才需要 Desk。

### 5.4 一切看起来正常但什么都不动

- 灯蓝色，`franka-remote status` 显示 servo 活动，`franka_link.txt` 里 `cmd_pkts_last_s` 是你的速率、`cmd_age_ms` 很小：servo 在有意 hold，要么是力/力矩 freeze（`freeze=1`），要么是目标等于当前位姿。等 1.5 s 或者移动目标。
- `cmd_pkts_last_s=0`：你的发送端没到达主机（IP 错误、锁、bridge 挂了）。
- `cmd_drop_allow` 或 `cmd_drop_latch` 在增长：源地址错误，或另一个发送端被 latch 了。
- Desk 里 FCI 已激活，但 servo 启动失败并给出 Desk 侧的消息：你激活 FCI 之后有人按了急停或取消激活了机器人。重做第 3 节的第 2-4 步。

## 6. 不要做的事

- servo 活动期间不要运行 Desk 应用、`goto`、`echo` 或手动引导：FCI 只允许一个客户端，后来者胜出，servo 会在运动中途死掉。
- 不要靠关掉浏览器标签页来停止 FCI。用 **Deactivate FCI**，或者就让它开着；无论怎样 token 都不随标签页消失。
- 灯还亮着时不要按电源开关关闭控制柜。先从 Desk 关机。
- 不要让机械臂处于解锁且无人看管的状态。锁定关节或者关机。
