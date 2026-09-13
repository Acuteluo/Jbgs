# Jbgs 涵洞巡检机器人感知系统

DG-202603 涵洞巡检赛题感知系统：双大恒相机（左/右）+ 红外热像 + 塔石环境传感器 + 核心处理同屏显示 + 巡检通信协议。ROS2 Humble，C++ 为主，launch 为 Python。

## 唯一启动命令

```bash
./run.sh
```

一键完成：定位工程根 → 可控干净重建（删除 `build/ install/ log/` 后 `colcon build --symlink-install`）→ source 环境 → 启动总 launch（双大恒 + 红外 + 传感器 + core_node 同屏 + 巡检协议）。

- 退出：终端按 **Ctrl+C** 一次，整组进程停止（脚本有兜底收尾）。
- 只构建不启动：`./run.sh --build-only`
- launch 参数透传：`./run.sh sim:=false`、`./run.sh show_windows:=false in_trulyworking:=false` 等。
- 无图形环境（无 DISPLAY）时同屏窗口自动强制关闭，不会崩溃。

## 总配置 config/launch.json

| 键 | 说明 | 默认 |
|---|---|---|
| `show_windows` | 是否弹出同屏大面板（左/右大恒检测叠加 + 红外叠加 + 传感器数据） | `true` |
| `fullscreen` | 面板全屏展示：窗口无边框占满整屏，窗格高度按屏幕宽高比自动计算（画布与屏幕同比铺满无黑边，相机图像等比居中留边） | `true` |
| `in_trulyworking` | 巡检模式总开关；false 时巡检节点不订阅不处理不保存 | `true` |
| `sim_mode` | 热插拔模拟总开关（无真机时置 true） | `true` |
| `log_level` | ROS 日志级别 debug/info/warn/error | `info` |
| `save_dir` | 巡检图片保存目录（相对工程根） | `run_save` |
| `input_topic` / `ack_topic` | 巡检指令输入 / 确认输出话题（UInt8） | `/inspection/command` / `/inspection/ack` |
| `frame_timeout_sec` | 巡检目标帧等待超时 | `3.0` |
| `target_frame_index` | 取 t 后第 N 帧 | `2` |
| `enable_yolo` / `enable_ir_seepage` / `enable_sensor` | 模型与传感器开关 | `true` |
| `camera_config_files` | 左/右/红外相机设备 JSON 路径 | 见下 |
| `sim.*` | 模拟热插拔时间窗（初始缺失/中途断连时刻与时长，每设备独立） | 见文件 |

相机设备定义：`config/galaxy_camera1_.json`（左）、`config/galaxy_camera_2.json`（右）、`config/infrared_camera.json`（红外）。含设备 ID（serial_number/user_id/ip/device_index，可留空）、别名、内参/分辨率占位。**真机未接入时不要伪造设备 ID**：留空或用默认，启动后从日志中抄录实际 SN/IP 回填。

launch 参数优先级高于 launch.json。**全部可调 launch 键**（空值 = 尊重 launch.json/yaml）：

| 键 | 说明 |
|---|---|
| `sim` | true=全模拟 / false=真机 |
| `show_windows` | 同屏面板开关 |
| `in_trulyworking` | 巡检协议开关 |
| `sensor` | 是否启动传感器驱动 |
| `input_topic` / `ack_topic` | 巡检指令/确认话题 |
| `save_dir` | 巡检图片保存目录 |
| `frame_timeout` | 巡检目标帧超时（秒） |
| `yolo` / `ir_seepage` | 左右 YOLO / 红外渗水检测开关 |
| `absent` / `disc_at` / `disc_dur` | 双大恒统一初始缺失 / 断连时刻 / 断连时长（秒，none=禁用断连） |
| `left_absent` `left_disc_at` `left_disc_dur` | 左大恒独立时间窗（覆盖统一值） |
| `right_absent` `right_disc_at` `right_disc_dur` | 右大恒独立时间窗 |
| `ir_absent` `ir_disc_at` `ir_disc_dur` | 红外独立时间窗 |
| `sensor_absent` `sensor_disc_at` `sensor_disc_dur` | 传感器独立时间窗 |
| `log_level` | debug/info/warn/error |

示例：`./run.sh left_absent:=30 left_disc_at:=40 left_disc_dur:=5`（左相机 30s 才接入、40s 起断连 5s）。

## 巡检通信协议（仅本机 ROS2 topic，无串口）

包 `culvert_inspection`，消息 `std_msgs/msg/UInt8`：

1. 输入话题收到 `0x01`：记录单调时刻 t，对左右相机**分别独立**等待"t 后按**到达顺序**的第 2 帧"（t 时刻视为第 0 帧；不用消息时间戳做选择，时间戳只用于文件名）。
2. 目标帧解码失败或相机离线 → 继续等该相机的第 3、4…帧；自最近一次成功捕获（或触发）起超过 `frame_timeout_sec` → 放弃，**不发确认**（持续坏帧也会被超时兜住）。
3. 左右均取到 → 写盘 `run_save/<该帧时间戳>_left.jpg`、`run_save/<该帧时间戳>_right.jpg`（纳秒时间戳，左右各自独立）→ **恰好发布一次 `0x02`**。
4. 重复 `0x01`、Armed/保存中再次 `0x01`（try_lock 拦截）、超时、写盘失败：均有明确状态机行为与限流日志，绝不误发 `0x02`；写盘单侧失败会清理已写出的半对文件。
5. `in_trulyworking=false`：节点不订阅协议话题、不保存任何巡检图片。

状态机核心在 `src/culvert_inspection/include/culvert_inspection/inspection_fsm.hpp`（无 ROS 依赖，gtest 单测覆盖）；rclcpp 薄封装在 `src/inspection_node.cpp`。

## 话题总表

| 话题 | 类型 | 说明 |
|---|---|---|
| `/left_camera/image_raw/compressed` | CompressedImage | 左大恒 JPEG |
| `/right_camera/image_raw/compressed` | CompressedImage | 右大恒 JPEG |
| `/ir_camera/image_raw/compressed` | CompressedImage | 红外 JPEG（原生 256x192，勿放大检测） |
| `/sensor/env` | sensor_interfaces/EnvData | 温湿度/CO2（1Hz） |
| `/core_node/status` | String | 1Hz 心跳：各路帧率 + on/OFF 离线标识 + 传感器摘要 |
| `/core_node/save_image` | Empty | 调试：按 S 键保存总图 |
| `/inspection/command` | UInt8 | 巡检指令（0x01） |
| `/inspection/ack` | UInt8 | 巡检确认（0x02） |
| `/inspection_node/status` | String | 巡检状态机状态（10Hz） |

## 解耦与热插拔设计

- 任一设备初始缺失、运行中断连、重新接入，都不会阻塞/退出/重启其他设备；离线路径与真机走同一套"超时判定 → 重连"代码。
- 取图、左右 YOLO 推理（双实例并行）、红外渗水检测（背景差分）、传感器轮询、显示、巡检保存全部独立线程/回调组；每级只保留最新帧，慢消费者丢旧帧不回压。
- 同屏面板每个相机窗格信息条实时显示两个帧率：`src=` 取原相机原图帧率（帧间到达间隔 EMA，验证取图是否正常）、`det=` 模型处理完画框后的输出帧率（解释窗口刷新快慢；推理慢时 src 不变、det 下降，一眼定位瓶颈在拿图还是在模型）。
- 面板为全屏窗口（`fullscreen`，默认开）：屏幕分辨率经 xrandr 自动探测（失败时用 `screen_width`/`screen_height` 参数兜底），信息互不遮挡：窗格标题与帧率在顶部信息条、传感器块在其下方、总图右下角为显示刷新率。
- 传感器数据全部打印：T/RH/CO2 三值 + 温湿度/CO2 逐字段有效位（TH=ok/FAIL, CO2=ok/FAIL）+ 数据年龄 age；数据陈旧超过 3s 按离线显示。
- 离线标识：core_node 状态行 `left_hz=0(OFF)`、`env[no_data]`；驱动日志 2~5s 节流告警。

## 模拟测试

```bash
bash tests/run_tests.sh          # 全部（含干净重建，约 8 分钟）
bash tests/run_tests.sh 05       # 只跑巡检协议
colcon test --packages-select culvert_inspection   # 状态机 gtest 单测
```

系统测试（`tests/CMakeLists.txt`，C++ + ctest，全部基于模拟，不依赖真机与外部数据集）：

| 用例 | 覆盖 |
|---|---|
| `01_clean_build` | 干净重建成功 |
| `02_full_sim` | 全模拟启动，全部话题可观测，关停无残留 |
| `03_missing_{left,right,ir,sensor}` | 初始缺任一设备，其余持续工作 + 离线标注 |
| `04_hotplug` | 四设备错峰断连/恢复、其他设备不受扰、OFF 标注、节流日志 |
| `05_protocol` | 严格选 t 后第 2 帧（含文件内容校验）、重复 0x01、解码失败顺延、单侧离线超时、写盘失败、恢复后独立巡检 |
| `07_off_mode` | in_trulyworking=false 不处理不保存 |

单测（gtest，`colcon test`，12 例）另覆盖：触发时刻=第 0 帧、可配置目标帧序号、持续坏帧仍超时（解码失败不计进展）、外部计数路径不重复计数、进展与超时边界、半对文件清理等。

## 目录结构

```
run.sh                  一键入口
AGENTS.md / README.md   说明
config/                 launch.json + 三相机设备 JSON
src/                    ROS2 包: galaxy_camera_dual, ir_camera_driver,
                        tas_sensor_driver, culvert_core, culvert_inspection,
                        sensor_interfaces, bringup
tests/                  C++ 系统测试(CMake+ctest) + run_tests.sh
env/env.sh              环境装载(run.sh 复用)
models/                 ONNX 模型(勿删)
pictures/               模拟轮播实拍帧(勿删)
scripts/                辅助脚本
run_save/               巡检图片输出
Culvert-Visual-Inspection-main/   红外参考工程(有 COLCON_IGNORE, 不参与构建)
```

## 真机接入待办

0. **推荐：先跑配置向导**（交互式，自动完成下面 1~3 的设备确认与写配置）：

   ```bash
   source install/setup.bash
   ros2 run galaxy_camera_dual camera_setup_wizard          # 交互式
   ros2 run galaxy_camera_dual camera_setup_wizard --check  # 只检查不写入
   ```

   向导流程：① 枚举大恒相机（失败打印网卡网段/MTU/防火墙/SDK 排障清单）；② 逐台**全屏预览**两台大恒，测试者按 ENTER 结束预览并回答是左侧还是右侧，序列号自动绑定进对应 JSON；③ 逐个 /dev/video 候选预览确认红外热像（USB 直插直用，无需配置），写入 infrared_camera.json；④ 对每个串口候选发 Modbus 读数并显示 ~6 秒实时温湿度/CO2，确认后写入 sensor_driver.yaml；⑤ 四模块全部确认后自动把 launch.json 的 `sim_mode` 置 false。被修改文件均备份为 `*.bak`。

1. （手动方式）相机 JSON 回填两台大恒的 serial_number（首次真机启动后从日志抄录），`sim_mode` 置 false（或 `./run.sh sim:=false`）。
2. 红外 `config/infrared_camera.json` 填 device_path/device_index；确认 `/dev/video*` 权限。
3. 传感器 `src/tas_sensor_driver/config/sensor_driver.yaml` 确认 ttyUSB 与 Modbus 地址；`sim_mode: false`。
4. 标定后回填三相机内参（JSON intrinsics + 包内 camera_info yaml）。
5. 检查网卡 MTU 与巨帧（camera JSON packet_size 8192 需 MTU≥9000 或改小）。

## 已知限制

- 真机尚未验证：相机曝光/触发时序、红外 UVC 帧率、串口时序均只在模拟/代码路径层面对齐。
- 巡检"t 后第 2 帧"以巡检节点**到达顺序**为准；DDS 传输有毫秒级延迟，测试用静默间隔消除边界歧义。
- 保存中的 0x01 用 try_lock 拒绝（不排队）；周期结束后再来的 0x01 视为新一轮巡检。
- Release 构建类型未启用：-O3 会误报 galaxy SDK 封装的 stringop-overflow。
- core_node 同屏面板在无 DISPLAY 环境强制关闭（状态可经 `/core_node/status` 观测）。
