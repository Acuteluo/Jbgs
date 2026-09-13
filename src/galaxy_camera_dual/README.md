# galaxy_camera_dual

> **2026-09 更新提示(以本节为准, 下文历史文档部分细节已过时):**
> - 新增 `sim_mode` 模拟模式(无实体相机时热插拔全链路测试, 插拔语义与真机一致);
> - 修复两处热插拔缺陷: ①0 台设备时构造函数死循环(初始断连永远无法恢复)
>   → 改为有限次枚举后交给热插拔守望线程; ②打开失败后 `opened` 标志误置 true;
> - 话题精简: 每台相机默认只发布 `<camera_name>/image_raw/compressed` 一条
>   (camera_info 默认不再发布, 需要时 `publish_camera_info: true`);
> - 启动参数支持 `sim:=true/false` 覆盖 yaml。
> 系统级说明见工程根目录 `README.md`（原 `COMMANDS.md` 已并入其中）。

基于大恒(Galaxy) SDK 的 ROS2 多相机驱动：**一个节点启动并管理两台 MER-500-14GC GigE 相机**（涵洞检测车左/右两侧 ±30° 安装），也可退化为单相机模式。

本包由 `rm_vision_ros2_galaxy_camera` 改造而来，原包文件未被修改；本包完全独立。

## 与原包的关键差异（对症下药的部分）

| 问题 | 本包方案 |
| --- | --- |
| 原包硬编码 `GXOpenDeviceByIndex(1)`，只能开一台 | 每台相机可分别用 **序列号 / UserID / IP / 设备序号(从 1 开始)** 寻址 |
| `GXInitLib`/`GXCloseLib` 是进程级全局调用，多实例并发会冲突 | `GalaxyDevice` 内做**引用计数 + 互斥**，多个节点/设备可共存于同一进程 |
| 采集线程与参数回调并发操作同一个句柄无保护 | `GalaxyDevice` 所有 SDK 调用**逐设备互斥**，采集线程与执行器线程不会竞争 |
| 相机掉线直接 `rclcpp::shutdown()` 拖死整个进程 | 单台相机**自动重连**（连续超时或 SDK 掉线错误 → 停采/重开/重采），残帧只丢弃不重连 |
| 参数回调 success 标志写反的 bug | 已修正；`exposure_time`/`gain` 可在线动态调整 |
| 只有一个 image_raw 话题 | 每台相机独立话题：`<camera_name>/image_raw` + `<camera_name>/camera_info`（默认 `left_camera/...`、`right_camera/...`），独立 frame_id 与标定文件 |
| 图像时间戳用 ROS 接收时刻（抖动大，影响与里程计融合） | 默认使用**相机硬件 tick** 映射到 ROS 时间（`use_camera_timestamp: true`），低抖动 |

## 多线程模型

```
galaxy_camera_dual_node (单进程)
├── MultiThreadedExecutor           ← 参数回调 / 服务(执行器线程, 不与取图争抢)
├── 采集线程 #1 (left)  ──┐
│   GXGetImage(阻塞) → Bayer→RGB → publish
└── 采集线程 #2 (right) ──┘
    每个 GalaxyDevice 一把互斥锁 + 原子停止标志
    关闭时: running=false → 停采(唤醒阻塞的取图) → join → 关设备 → 关库
```

- 每台相机的阻塞 `GXGetImage` 在**自己的线程**里，互不阻塞，也不会阻塞 ROS 执行器；
- 节点仍注册为 component，可进 composition 容器；`libgalaxy_camera_dual.so` 内的库初始化是引用计数的，容器里多实例也安全；
- 相机掉线只由它自己的采集线程重连，另一台照常发布。

## 构建

```bash
mkdir -p ~/galaxy_ws/src
cp -r ~/galaxy_camera_dual ~/galaxy_ws/src/
cd ~/galaxy_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select galaxy_camera_dual
source install/setup.bash
```

## 第一次使用：先认清左右相机

先不填序列号，用默认的 `device_index: 1 / 2` 空跑一次：

```bash
ros2 launch galaxy_camera_dual dual_camera.launch.py
```

启动日志会打印每台设备的 **序号/型号/SN/UserID/IP/MAC**：

```
Found 2 device(s):
  [1] model=MER-500-14GC SN=DBK1234567 UserID= IP=192.168.1.101 MAC=...
  [2] model=MER-500-14GC SN=DBK7654321 UserID= IP=192.168.1.102 MAC=...
```

把左侧相机的 SN 填进 `config/camera_params.yaml` 的 `cameras.left.serial_number`，右侧填进 `cameras.right.serial_number`。**强烈建议用序列号寻址**：按序号寻址依赖枚举顺序，重启后两台同型号相机顺序可能互换，左右会错位。

> 提示：也可以利用 `user_id_to_set` 参数把相机烧写成 "left"/"right"（持久化在相机里），之后用 `user_id: left` 寻址。

## 参数说明（config/camera_params.yaml）

全局：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `camera_names` | `[left, right]` | 相机组名列表，每组对应 `cameras.<name>.*` |
| `use_sensor_data_qos` | `false` | 使用 sensor_data QoS |
| `use_camera_timestamp` | `true` | 用相机硬件 tick 打时间戳（曝光开始瞬间） |
| `timestamp_drift_compensation` | `true` | 用 EMA 补偿相机晶振与主机时钟的漂移（推荐开） |
| `timestamp_exposure_center` | `false` | 时间戳平移至曝光中心（+曝光/2，运动补偿场景可开） |
| `grab_timeout_ms` | `300` | `GXGetImage` 超时 |

每相机（`cameras.<name>.*`）：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `serial_number` | `""` | 按序列号打开（最高优先级） |
| `user_id` | `""` | 按 UserID 打开 |
| `ip_address` | `""` | 按 IP 打开 |
| `device_index` | 1 / 2 | 按序号打开（GxIAPI 从 **1** 开始；0=不用） |
| `user_id_to_set` | `""` | 非空则烧写为相机持久 UserID |
| `camera_name` | `left_camera` / `right_camera` | 话题与 camera_info 名字 |
| `frame_id` | `left_camera_optical_frame` / ... | 图像 frame_id |
| `camera_info_url` | 对应 yaml | 标定文件（左右各一份，先用占位值，**需分别标定后替换**） |
| `exposure_time` | `5000` (us) | 曝光，运行时可调 |
| `gain` | `1.0` | 增益，运行时可调 |
| `frame_rate` | `0.0` | Hz；0=连续采集，>0 时使用软件触发精确限帧 |
| `packet_size` | `0` | GigE 包长；主机 MTU=9000 时推荐 8192 |
| `throughput_limit_bps` | `0` | 设备链路带宽上限（B/s）；双机 10 Hz 共用千兆链路时每台推荐 55000000 |
| `packet_delay` | `0` | 可选 GVSP 包间隔（相机 tick）；默认关闭，需按具体固件实测后设置 |

在线调参：

```bash
ros2 param set /galaxy_camera_dual cameras.left.exposure_time 8000.0
ros2 param set /galaxy_camera_dual cameras.right.gain 2.0
```

## 话题

- `/left_camera/image_raw`、`/left_camera/camera_info`
- `/right_camera/image_raw`、`/right_camera/camera_info`
- 查看：`ros2 topic hz /left_camera/image_raw`

## 时间戳与多传感器匹配

**时间戳的物理含义**：相机在"曝光开始（帧开始）"瞬间把内部高精度计数器（tick）锁存进 `nTimestamp`，程序把它换算成 ROS 时间。帧间间隔由相机晶振决定（微秒级抖动），远优于"软件收到图像的瞬间"。

- 相机晶振与主机时钟有 ppm 级频差，长时间运行会累积漂移 → `timestamp_drift_compensation: true`（默认）用 EMA 平滑"到达时刻−相机侧时刻"的偏差，让时间戳长期跟随主机时钟，与里程计/IMU 对齐；
- MER-500-14GC 是全局快门，整帧同时曝光 → 运动补偿/SLAM 场景可开 `timestamp_exposure_center: true`，时间戳平移至"曝光中心"（曝光开始 + 曝光时间/2）；
- 每帧的 `camera_info` 时间戳与图像完全一致，下游取标定无需再配对。

**左/右相机对齐**：

- 两台相机自由运行时帧时刻不会天然对齐。用 `message_filters` 的 `ApproximateTimeSynchronizer`，容差（slop）取半帧周期（如 10 Hz → 50 ms），即可把同一时刻附近的左右帧配对；
- 若要严格同步（微秒级），用硬件触发：MER-500-14GC 支持外部触发（Line0），两台的 Trigger 输入接同一路触发信号（或把一台的 Strobe 输出接到另一台的 Trigger 输入），两台同时曝光，之后用 `TimeSynchronizer` 精确配对即可。

**与里程计/IMU 对齐**：同样按时间戳用 `message_filters` 同步；若里程计频率远高于图像，常见做法是取最邻近时间戳的姿态，或在两帧姿态之间按图像时间戳线性插值。图像话题默认 sensor_data QoS，与传感器数据兼容。

## MER-500-14GC 部署注意事项（涵洞裂缝检测）

1. **带宽**：500 万彩色相机全速约 70 MB/s/台，两台 ≈ 140 MB/s，超过千兆网口（约 125 MB/s）。双机 10 Hz 共用单条千兆上联时，使用 `frame_rate: 10.0`、`throughput_limit_bps: 55000000`、`packet_delay: 0`、主机 MTU 9000 与 `packet_size: 8192`；若仍有残帧，则改用**两块独立网卡**（或 2.5G 上联），也可降低分辨率/ROI。
   - 相机与网卡同一网段（程序用 `GXUpdateAllDeviceList` 可枚举所有网段，但同网段最稳）。
2. **照明**：涵洞内基本无环境光，裂缝检测需要均匀补光，曝光/增益根据实测在参数里调（夜间裂缝目标可用更大曝光 + 白光补光）。
3. **镜头**：2.8" 6mm 定焦，安装角 ±30° 已由机械结构固定，标定时**左右相机各自标定**，替换 `camera_info_left.yaml` / `camera_info_right.yaml`。
4. **时间戳**：与里程计/IMU 融合时保持 `use_camera_timestamp: true`。
5. **重连**：施工环境易松线，单台掉线会自动重连，重连后会自动重打时间基准并恢复曝光/增益/帧率设置。

## 单相机模式

```bash
ros2 launch galaxy_camera_dual galaxy_camera.launch.py
```

话题为 `/narrow_stereo/image_raw`，与原包行为兼容。

## rqt 看图与帧率(实测结论)

- 相机满帧 14fps; 每帧 raw 消息 15MB 是发布环节的主要负担(原包同样受限);
- 本包额外发布 JPEG 话题 `<camera_name>/image_raw/compressed`(默认开启,
  `publish_compressed`/`jpeg_quality` 可调), 实测 **14.0 fps 满帧稳定**;
- rqt_image_view 里选 `/narrow_stereo/image_raw/compressed` 即可流畅看图;
  raw 话题留给检测算法(无损);
- 若 raw 帧率异常低: 多半是有残留实例占着相机, 先 `pkill -x galaxy_camera_d`
  再启动; 同一时刻只能跑一个相机驱动实例。
