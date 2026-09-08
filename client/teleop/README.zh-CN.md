[English](README.md) | 中文

# Quest / WebXR 遥操作

`bin/franka-teleop live` 先在 RT host / 实时主机上启动位姿 servo，再在本机启动 `02_webxr_to_franka.py`。bridge 通过 HTTPS 在 4443 端口提供 WebXR 页面（`teleop` pip 包，页面来自 `frontend_swapped/`），以约 90 Hz 接收手柄位姿。它施加下面的防护规则，并把 128 字节的 `O_T_EE` 目标发往 `FRANKA_SERVO_HOST:50001`。逐帧数学：[`../../docs/GUIDE.zh-CN.md`](../../docs/GUIDE.zh-CN.md) 第 4.5 节。

```
Quest (Meta Browser, WebXR) --HTTPS/WebSocket :4443--> this PC: 02_webxr_to_franka.py
                                                            |  UDP 50001, 128 B, ~90 Hz
                                                            v
                                                 RT host: cartesian_pose_servo (1 kHz)
```

## 从头显访问 bridge

Quest 打开 `https://<this PC>:4443/`。两种可行方案：

1. 同一 WiFi：bridge 会打印 `https://<LAN IP>:4443/`。证书是自签名的（`teleop` 包里的 `cert.pem`）：在 Meta Browser 里接受一次警告，然后按 Enter VR。在本机防火墙放行 TCP 4443。
2. Tailscale Funnel（Quest 在另一个网络）：在本机执行 `tailscale funnel 4443`，把 URL 填进 `config.env` 的 `FRANKA_TELEOP_URL`，bridge 会打印它。Funnel 有有效证书，所以没有警告。

WebXR 需要安全源，因此只支持 HTTPS。`$FRANKA_PY teleop/01_webxr_pose_reader.py` 会打印每个回调（位姿、按键），不需要机器人和实时主机。

### 头显内的操作流程

1. **按 Enter VR 之前，站到机器人正前方、面朝机器人。** 会话原点在那一刻被捕获；之后所有手柄运动都在那个坐标系里解释（“前” = 从那个位置看向机器人的方向）。
2. 之后可以走动。映射保持在标定时的坐标系里：从侧面或从机械臂后方操作，它仍按你标定的方向运动。
3. **结束时，先在网页里按 “Exit VR”**，再从 PC 上停止 bridge 和 servo。会话仍开着时杀掉 bridge，会让头显继续往空处推流，并干扰下一次启动。
4. 按住 Trigger 移动（dead-man），按 Grip 切换夹爪。走动前先松开 Trigger。

## 手柄映射（`frontend_swapped/`）

| 控制 | 功能 |
|---|---|
| Trigger（食指），按住 | dead-man：只有按住时机器人才跟随手柄。松开后继续发送最后一个目标，所以机械臂保持位姿而不会漂移 |
| Grip（中指），按下 | 切换 Franka Hand 开 / 合（`bin/gripper_cmd` -> 主机上的 `franka-ctl gripper`） |
| A | 保留；只有 `--calibrate` 用它把当前 EE 采样到 `/tmp/franka_bbox_points.json` |
| B | 保留（本 bridge 中无功能） |

原版 `teleop` 前端是 Grip = 移动、Trigger = 夹爪；对调在 `frontend_swapped/index.html` 里（注释 "Trigger <-> Grip swapped"）。没有那个目录时，bridge 回退到原版页面并会提示。

## 映射与防护规则（全部在 `bin/franka-teleop` 里设置）

| 参数 | 默认值 | 含义 |
|---|---|---|
| `FRANKA_SCALE_TRANS`（`--scale`） | 1.0 | 手柄位移到机器人位移的比例。1.0 = 1:1，在胸口高度操作舒适；2.0 会把 Quest 的噪声也放大一倍 |
| `FRANKA_SCALE_ROT`（`--rot-scale`） | 0.5 | 手柄旋转到 EE 绕初始位姿旋转的比例；0.5 把手腕抖动减半 |
| `FRANKA_MAX_STEP`（`--max-step`） | 0.0055 m | 每个 90 Hz 帧的最大平移（0.5 m/s）。目标沿其方向被截断，从不丢弃，所以机械臂持续运动。0.02 m（1.8 m/s）曾触发 FR3 的关节速度限制 |
| 旋转步长 | 0.05 rad/帧（常量 `MAX_ROT_STEP_RAD`） | 旋转速率上限 |
| `FRANKA_WS_{X,Y,Z}_{MIN,MAX}`（`--ws-*`） | 参考桌面的盒子 | 机器人基座坐标系下的轴对齐工作空间盒（米）。目标被裁剪到盒内，并重置 WebXR 锚点，所以操作者在边缘处必须松开并重新按下 Trigger |
| mirror 模式 | 开（`--no-mirror` 关闭） | 操作者面对机器人：目标绕经过初始位姿的基座 z 轴旋转 180 度，这样手柄上的“左”就是操作者看到的“左” |

初始位姿来自 `/tmp/franka_init_pose.txt`（由 mirror 在每个 servo 实例启动时写一次）。如果它落在工作空间盒之外，bridge 以 2 退出：先执行 `franka-remote goto home`（或 `franka-teleop restart`），或者放宽盒子。

要测量你自己的盒子：`franka-teleop servo-only`，然后 `$FRANKA_PY teleop/02_webxr_to_franka.py --live --calibrate --udp-host $FRANKA_SERVO_HOST`。把机械臂拖到桌子四角和最高的有用点，在每个点按 A。读取 `/tmp/franka_bbox_points.json`，把最小/最大值（留出余量）填进 `config.env`。

## servo 侧如何处理这条数据流

servo 侧的防护（Trigger 那一行依赖的 200 ms hold、力/力矩冻结、力矩上限、reflex 恢复、发送者 latch）：[`../../docs/GUIDE.zh-CN.md`](../../docs/GUIDE.zh-CN.md) 第 2.3 节。只接受 `FRANKA_CLIENT_IP`；本机上的第二个 bridge 会通过 `franka_sender_lock.py` 以 75 退出，而不是被静默忽略。

servo 没有工作空间防护；本 bridge 里的盒子是机器人自身关节 / 速度限制之前唯一的软件限制。**把急停（user stop）放在手边。**
