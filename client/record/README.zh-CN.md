[English](README.md) | 中文

# Episode 录制

`record_episodes.py` 把 RealSense 帧和来自 `/tmp/franka_*.txt`（`franka_state_mirror` 维护的那些文件）的机器人状态写成每个 episode 一个 HDF5 文件。它与任务无关：不发命令，不了解任务，不生成派生标签。单独启动遥操作（`franka-teleop live`），在终端前台运行录制器，用键盘驱动 episode（`s` 开始，`e` 结束并保存，`d` 丢弃上一段，`q` 退出）。

```
python3 record/state_reader.py --selftest --secs 3        # EE path alive? (ee_ok=60/60 expected)
python3 record/selftest.py 3                              # cameras only, 3 s -> /tmp/*.h5
python3 record/record_episodes.py --out-dir ~/datasets/pick --prefix pick [--auto-end-secs 30]
```

环境：`requirements-record.txt`（pyrealsense2、h5py、numpy、opencv-python、PyYAML）。相机在 `config/cameras.yaml` 中声明（serial -> role）；没插上的 role 会被跳过并给出警告。`--cams role1,role2` 只录一个子集。

## 相机配置

`cameras/recorder.py` 在数据流打开时应用 RECORD 配置：自动曝光和白平衡收敛 60 帧，然后曝光、增益和白平衡在整个会话中冻结，并作为文件属性存储。对比度、gamma 和饱和度保持传感器默认值，亮度为 0，因此训练图像里不会烘焙进任何色调曲线。曝光被限制在帧周期的 60 %（30 fps 时为 20 ms），由增益补偿；否则暗场景会跌破目标帧率。`--exposure role=microseconds` 手动锁定某个 role（当收敛值让明亮物体过曝时）。`cameras/grabber.py` 是预览 / 快照版本；录制器不使用它。

这里观察到的 RealSense 行为。在没有 librealsense udev 规则的主机上，D455 执行 `hardware_reset()` 可能让设备停在 `Protocol error` 状态，直到重新插拔 USB 线（因此 YAML 里 `hardware_reset_on_start: false`）。快速重启 pipeline 也会以同样方式卡死相机。一台相机同一时间只能被一个进程打开。

## 数据集格式

每个 episode 一个文件，`<prefix>_epNN.h5`（编号接着输出目录中已有的文件继续），gzip 级别 1，在后台线程中写入。

| 路径 | 形状 / dtype | 含义 |
|---|---|---|
| `observations/<role>/image` | `(T, H, W, 3)` uint8，BGR | 一台相机的彩色帧，按帧分块 |
| `observations/<role>/hw_timestamp_ms` | `(T,)` float64 | 每帧的 RealSense 硬件时间戳（ms；相机自己的时钟，`frame.get_timestamp()`） |
| `observations/<role>/wall_timestamp_s` | `(T,)` float64 | 收到该帧时本机的墙钟时间（`time.time()`） |
| `observations/<role>` attrs | `serial`、`locked_exposure`、`locked_gain`、`locked_white_balance`、`locked_contrast`、`locked_gamma`、`locked_saturation`、`locked_brightness` | 冻结的传感器设置（-1 = 该传感器不支持） |
| `state/ee_pose` | `(N, 16)` float32 | servo 以 20 Hz 发布的 `O_T_EE`，列主序 4x4（平移在 12、13、14） |
| `state/ee_ok` | `(N,)` bool | 样本有效性（见下） |
| `state/_t` | `(N,)` float64 | 轮询时本机的墙钟时间（`time.time()`） |
| `state/wrench`、`state/wrench_ok` | `(N, 6)` float32、`(N,)` bool | `O_F_ext_hat_K`（Fx Fy Fz Tx Ty Tz），仅在 `--with-wrench` 时 |
| `state/joint_state`、`state/joint_ok` | `(N, 28)` float32、`(N,)` bool | `q dq tau_J tau_ext_hat_filtered`（各 7 个），仅在 `--with-joints` 时 |
| 根 attrs | `episode`、`duration_s`、`t_start_s`、`t_end_s`、`poll_hz`、`state_dir`、`ee_stale_ms`、`ee_ok_fraction`、`client_host`、`servo_host`、`config` | 记账信息 |

`T` 每台相机不同（各自的时钟，标称 30 fps）；`N` = 轮询率 x 时长（默认 20 Hz）。已在参考客户端上以双主机模式验证。文件里不做任何重采样或对齐。事后按墙钟对齐（每帧的 `wall_timestamp_s`，每个状态样本的 `_t`，同一个 `time.time()` 时钟）。`hw_timestamp_ms` 可以找出单台相机内的丢帧（`dt` 应为恒定的 33.3 ms）。

### `ee_ok`

`state_reader.py` 每次轮询都读取 `/tmp/franka_current_ee.txt`。只有当文件 mtime 距今不到 250 ms（五个 20 Hz 样本的间隔）且恰好包含 16 个浮点数时，`ee_ok = True`。否则 `ee_pose` 全为零且 `ee_ok = False`：servo 已停、链路静默或 mirror 挂了，这次轮询不携带任何机器人信息。生成标签时丢弃这些样本；不要跨它们插值。录制器会打印每个 episode 的 `ee_ok` 比例，低于 95 % 时告警。

### 客户端与主机之间的时钟同步

动作标签通常是相邻样本的 EE 差值（`ee_pose[k+1] - ee_pose[k]`）配上最近的图像。EE 样本在本机轮询时打上时间戳，但它是实时主机按自己的时钟产生的；mirror 的 `skew_ms`（主机 `t_real_ns` 与客户端接收时间之差）就是这个差距。在 20 Hz 下，10 ms 的偏差已经让样本错位五分之一步，标签也随之错位。保持两台机器 NTP 同步，让 `franka-client-preflight` 报告 `clock skew < 10 ms`；`/tmp/franka_mirror.txt` 的 `skew_ms` 字段显示当前值。背景：[`../../docs/GUIDE.zh-CN.md`](../../docs/GUIDE.zh-CN.md) 第 2.5 节。
