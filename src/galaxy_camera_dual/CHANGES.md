# galaxy_camera_dual 与原包(rm_vision_ros2_galaxy_camera)的改动清单

本包由原包整体复制改造而来, 原包文件未做任何修改。以下是逐项详细对比。

## 一、新增文件

| 文件 | 作用 |
| --- | --- |
| `include/galaxy_camera_dual/galaxy_device.hpp` | Galaxy SDK 的 C++ 线程安全封装(声明) |
| `src/galaxy_device.cpp` | 上述封装的实现(详见下文"galaxy_device 是什么") |
| `include/galaxy_camera_dual/galaxy_camera_node.hpp` | 多相机节点声明 |
| `config/camera_params_single.yaml` | 单相机模式的参数文件 |
| `config/camera_info_left.yaml` | 左相机标定文件(占位数据, 需实测替换) |
| `config/camera_info_right.yaml` | 右相机标定文件(占位数据, 需实测替换) |
| `launch/dual_camera.launch.py` | 双相机启动文件 |
| `CHANGES.md` | 本文件 |

## 二、galaxy_device 是什么(新增的核心文件)

原包直接在节点代码里裸调 GxIAPI(大恒 C 接口), 存在三个问题:

1. `GXInitLib()/GXCloseLib()` 是**进程级全局调用**, 一台相机一个节点的
   旧方案靠"一个进程只开一台"绕开冲突; 一旦一个进程管理两台相机
   (或 composition 容器里有多个节点实例), 就会互相干扰;
2. 同一台相机的 `GX_DEV_HANDLE` 会被**两个线程同时使用**: 采集线程里
   阻塞的 `GXGetImage()` 和参数回调线程里的 `GXSetFloat()`, 而 GxIAPI
   不保证并发安全;
3. 打开相机、读写特性等操作分散在节点代码里, 错误处理零散。

`galaxy_device.hpp/.cpp` 就是为解决这三点而加的**封装层**, 提供:

- 引用计数 + 互斥的 `GXInitLib/GXCloseLib` 管理(多设备/多节点安全);
- 每设备一把互斥锁, 所有 SDK 调用线程安全(采集线程与参数回调不竞争);
- 统一的寻址打开接口(序列号/UserID/IP/设备序号, 优先级固定);
- 线程安全的特性读写(整型/浮点/字符串/枚举/命令);
- 设备枚举(`listDevices`, 打印 SN/IP/MAC 用于区分左右相机)。

## 三、src/galaxy_camera_node.cpp 的逐项改动

| # | 原包 | 新包 | 原因 |
| --- | --- | --- | --- |
| 1 | 硬编码 `GXOpenDeviceByIndex(1)` 只开一台 | 每个相机组用 `serial_number/user_id/ip_address/device_index` 寻址 | 必须能分别定位两台同型号相机 |
| 2 | 一个节点 = 一台相机 = 一个采集线程 | 一个节点管理 N 台相机(默认 left/right), 每台一个独立采集线程 | 一台阻塞取图不影响另一台 |
| 3 | 话题固定 `image_raw` | 每台相机独立话题 `<camera_name>/image_raw` + `<camera_name>/camera_info` | 两台相机的图像/标定要分开 |
| 4 | 标定文件全局一份 | 每台相机一份(`camera_info_left.yaml` / `camera_info_right.yaml`), 独立 frame_id | 左右相机内参不同, 需分别标定 |
| 5 | `GXInitLib` 失败 → `RCLCPP_FATAL` + `exit(status)` 直接拖死进程 | 循环重试 + 打印错误, Ctrl+C 可优雅退出 | 相机晚于程序上电时也能自动恢复 |
| 6 | 取图连续失败 5 次 → `rclcpp::shutdown()` 杀死整个进程 | 只对该相机自动重连(停采→关设备→重开→恢复参数→重采) | 单台掉线不能影响另一台 |
| 7 | 参数回调 success 标志写反(`GX_SUCCESS` 时置 false) | 逻辑修正; 且 `exposure_time/gain` 为运行时参数, 其余只读 | 原逻辑反了, 在线调参必失败 |
| 8 | 时间戳用 `this->now()`(ROS 接收时刻) | 默认用相机硬件 tick(曝光开始瞬间锁存)换算 ROS 时间(`use_camera_timestamp` 可关) | 帧间间隔微秒级抖动, 与里程计/IMU 融合更平滑 |
| 9 | 无时钟漂移处理 | `timestamp_drift_compensation`(默认开): EMA 平滑"到达时刻−相机侧时刻"偏差, 长期跟随主机时钟 | 相机晶振与主机时钟 ppm 级频差, 长跑会累积毫秒级漂移 |
| 10 | 时间戳停在曝光开始 | `timestamp_exposure_center`(可选): +曝光时间/2 平移到曝光中心 | 全局快门相机运动补偿/SLAM 的标准约定 |
| 11 | `GXGetImage` 超时固定 500ms | `grab_timeout_ms` 参数(默认 300ms), 兼顾响应与关机速度 | 可调 |
| 12 | 无帧率控制 | `frame_rate` 参数(>0 时开启 SDK 帧率控制) | 两台 500 万彩色相机共用千兆网卡必须限帧 |
| 13 | 无 GigE 健壮性设置 | 命令超时 1000ms + SDK 采集队列 5 帧 | 网络抖动/消费波动时更稳 |
| 14 | 无 UserID 管理 | `user_id_to_set` 参数可把相机烧写为 "left"/"right" | 之后用 user_id 寻址, 不受枚举顺序影响 |
| 15 | 取图缓冲只用 `reserve`(data() 不可靠) | 用 `resize` 精确分配 payload 大小, 重连后重新校验 | 防止潜在越界 |
| 16 | 不检查帧状态 | 检查 `nStatus == GX_FRAME_STATUS_SUCCESS`, 丢弃残帧 | GigE 丢包会产生残帧 |
| 17 | 残帧累计 5 次会关闭相机并重连 | 残帧只丢弃和统计；连续超时/SDK 掉线错误才重连 | 避免双机拥塞触发重连风暴 |
| 18 | 双机只有软触发错相，UDP 突发仍可能重叠 | 新增 `throughput_limit_bps` 设备端流量整形；`packet_delay` 仅保留为按固件实测的可选项，默认关闭 | 避免固定包间隔使 10Hz 隔次漏触发 |
| 19 | 节点 main 单线程执行器 | `EXECUTOR MultiThreadedExecutor`(CMakeLists.txt) | 参数回调/服务与取图线程真正并行 |
| 20 | 依赖 SDK 默认的丢包恢复设置 | 显式开启 `GX_DS_ENUM_RESEND_MODE` | 少量偶发丢包可由相机重传救回 |

## 四、时间戳管理(2026-08 追加)

- 物理含义: `GX_FRAME_DATA.nTimestamp` 是相机在**曝光开始(帧开始)瞬间**
  锁存的硬件 tick 值, 换算公式 ROS时刻 = 首帧基准 + (tick差值/tick频率);
- 帧间间隔由相机晶振决定(微秒级抖动), 与网络延迟、CPU 调度无关;
- 相机晶振与主机时钟存在 ppm 级频差 → `timestamp_drift_compensation`
  (默认开)用 EMA(α=0.02)平滑"到达时刻−相机侧时刻"的偏差并回补,
  时间戳长期跟随主机时钟;
- 可选 `timestamp_exposure_center` 把时间戳平移到曝光中心(+曝光/2),
  适配全局快门的运动补偿场景;
- 重连后自动重建时间基准并重置漂移估计, 断线前后时间戳不会串台;
- 每帧 `camera_info` 与图像时间戳严格一致。

## 五、CMakeLists.txt 的改动

- 包名改为 `galaxy_camera_dual`, 可执行文件 `galaxy_camera_dual_node`;
- 增加 `galaxy_device.cpp` 参与编译, 安装 `include/` 与 SDK 头文件;
- `rclcpp_components_register_node` 增加 `EXECUTOR MultiThreadedExecutor`。

## 六、config / launch 的改动

- `camera_params.yaml`: 原包只有 camera_name/exposure/gain 三个参数,
  新包为全局参数 + `cameras.left.*` / `cameras.right.*` 两组相机参数;
- 新增 `camera_params_single.yaml`(单相机模式)与两份标定文件;
- 新增 `dual_camera.launch.py`(双相机); 原 `galaxy_camera.launch.py`
  改为指向单相机参数文件, 保持旧包兼容。
