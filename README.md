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

## 新机开箱配置（camera_setup_wizard）

新机/换机/更换任一相机后，只需运行一次交互向导，无需手工配 IP/网口/JSON：

```bash
cd <工程根> && source env/env.sh
ros2 run galaxy_camera_dual camera_setup_wizard          # 交互式配置
ros2 run galaxy_camera_dual camera_setup_wizard --check  # 只检查不写入(验收)
ros2 run galaxy_camera_dual camera_setup_wizard --tune-fps  # 只测/写帧率(换口后)
```

构建、启动、测试前必须退出 conda/venv（`env.sh` 会自动 `conda deactivate`）。向导里 sudo 密码只用于网络配置，不落盘。

向导自动完成：

1. **枚举大恒 GigE 相机**（跨网段广播）。优先按 SDK 报告的网卡 MAC 反查所在网口；若该口无载波（交换机已从扩展坞网口改插板载），则忽略坞以太网、改用有载波的同网段网口。红外/传感器仍可走坞上 USB。
2. **网络自动修复**：GxGVTL 以网卡**首个可用 IPv4** 为控制源。相机出厂 LLA 的 `169.254.x.x/16 scope link` 是合法控制地址，不会被误改；仅在首地址不在相机网段时才重排。其余地址（如 mid360 的 `192.168.1.50/24`）原序保留（闪断约 1 秒），并写回 NM（manual + MTU 9000）。
3. **突发容量核验**：USB2 坞用 MTU 1500；USB3/板载用 MTU 9000。RX ring 调到驱动上限（RTL8125B 上限 256，到顶只提示）。
4. **双机带宽/帧率**：按真实链路（USB2 坞 / USB3 坞 / 板载直插）双机同时软触发测流，写入能稳跑的最高 `frame_rate_hz`。USB 拓扑按整条父链的最低速率判定；相机交换机若经 USB2 或只协商到百兆，向导**拒绝写入任何双 5MP 帧率**，只提示换 USB3+ 或直插板载网口。USB3 坞、板载口和 2.5G 上联则由完整帧实测选择最大值；临界失败自动复测一次。相机未能打开/未进入采集（被占用、刚重插等）只报访问故障并保留既有 JSON，绝不误降帧率；红外/传感器可继续挂扩展坞。
5. **左右相机绑定**：逐台全屏预览（窗口拿不到焦点时终端直接回车等效，SSH 可用；输入 s 回车=跳过），帧数 ≥5 才允许确认，防止把无流相机绑进配置。
6. **红外确认**：逐个 /dev/video* 预览（等待热像开机 FFC 黑帧期，不强设分辨率）。
7. **传感器确认**：对 /dev/ttyUSB*/ttyACM* 逐个 Modbus 实读温湿度/CO2。
8. 确认结果写回三张设备 JSON（先备份 `.bak`）；全部确认后把 `launch.json` 的 `sim_mode` 置 false——存在未确认模块时保持不变，之后可再次运行向导补配。

`--check` 模式全程只读：枚举 → 网段/首地址 → MTU/RX ring → 双相机 3 秒真实取流残帧计数，任何一项不合格都明确拒绝并给出原因（防"能枚举≠能取流"的假绿）。

### 常见故障速查

| 现象 | 向导表现 | 处置 |
|---|---|---|
| 相机未识别（枚举 0 台） | 打印排障清单（网线/PoE、网段、巨帧、防火墙）后以失败退出，不假成功 | 查网线与 PoE 供电；`ip -br addr` 看相机网口是否有地址；临时 `sudo ufw disable` 排除防火墙 |
| 能枚举、open 超时 -14 | 诊断"首个可用地址不在相机网段"，自动把相机网段地址调到网卡首位 | 手工修：`sudo ip addr add 169.254.100.1/16 dev <相机网卡>`，且必须排在该网卡 IPv4 地址列表首位 |
| 预览 NO FRAME / 残帧 | `--check` 用双机残帧计数验收；向导按链路选包长 | 固定「恰好 5 张残帧然后断流」是 GXDQBuf 声明错误（已修）。RTL8125B ring 上限 256；板载直插稳 10fps |
| USB 串口总打不开 | 提示 brltty / ModemManager 抢占 | `sudo systemctl mask --now brltty brltty-udev ModemManager` 后重插 USB |
| 相机改接 USB 扩展坞（RTL8153） | 按 USB 父链最低速率打印 USB2/USB3/板载类型；**相机坞必须以 USB3+ 上联** | USB2/百兆链路直接拒绝写双 5MP 配置，提示换 USB3+ 或交换机直插板载；红外/传感器可继续走坞 USB；交换机改插板载后向导忽略坞上网口残留地址 |
| SDK 报已拔掉的坞网口 | 提示无载波已忽略，改用板载有载波网口 | 清掉坞以太网残留 `169.254`/`192.168.1.50`，勿动 mid360 所在的活网口 |
| 相机与 mid360 共用交换机/网卡 | 重排地址时 mid360 地址**原序保留**，仅闪断约 1 秒 | 无需处理；mid360 的静态地址属导航侧配置，不归向导管，重装系统后需手工补回 |

## 总配置 config/launch.json

| 键 | 说明 | 默认 |
|---|---|---|
| `show_windows` | 是否弹出同屏大面板（左/右大恒检测叠加 + 红外叠加 + 传感器数据） | `true` |
| `fullscreen` | 面板全屏展示：窗口无边框占满整屏，三路窗格按屏幕逻辑像素三等分；自动处理 Qt 高 DPI 缩放（相机图像等比居中留边） | `true` |
| `in_trulyworking` | 巡检模式总开关；false 时巡检节点不订阅不处理不保存 | `true` |
| `sim_mode` | 热插拔模拟总开关（无真机时置 true） | `true` |
| `log_level` | ROS 日志级别 debug/info/warn/error | `info` |
| `save_dir` | 巡检图片保存目录（相对工程根） | `run_save` |
| `input_topic` / `ack_topic` | 导航→视觉指令 / 视觉→导航状态话题（UInt8 电平, 20Hz） | `/vision_capture_cmd` / `/vision_capture_status` |
| `frame_timeout_sec` | 巡检目标帧等待超时 | `3.0` |
| `target_frame_index` | 取 t 后第 N 帧 | `2` |
| `enable_yolo` / `enable_ir_seepage` / `enable_sensor` | 模型与传感器开关 | `true` |
| `camera_config_files` | 左/右/红外相机设备 JSON 路径 | 见下 |
| `sim.*` | 模拟热插拔时间窗（初始缺失/中途断连时刻与时长，每设备独立） | 见文件 |

相机设备定义：`config/galaxy_camera_1.json`（左）、`config/galaxy_camera_2.json`（右）、`config/infrared_camera.json`（红外）。含设备 ID（serial_number/user_id/ip/device_index，可留空）、别名、内参/分辨率占位。**真机未接入时不要伪造设备 ID**：留空或用默认，启动后从日志中抄录实际 SN/IP 回填。

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
| `/vision_capture_cmd` | UInt8 | 导航→视觉指令（0x00 空闲 / 0x01 触发, 20Hz 电平） |
| `/vision_capture_status` | UInt8 | 视觉→导航状态（0x00 空闲 / 0x01 保存中 / 0x02 拍完） |
| `/inspection_node/status` | String | 巡检状态机调试状态（ARMED/SAVING/IDLE…） |
| `/core_node/left_annotated` `/right_annotated` | CompressedImage | 模型处理完画框后的标注帧（窗格分辨率、含信息条；有订阅者时才编码） |

## 解耦与热插拔设计

- 任一设备初始缺失、运行中断连、重新接入，都不会阻塞/退出/重启其他设备；离线路径与真机走同一套"超时判定 → 重连"代码。
- 取图、左右 YOLO 推理（双实例并行）、红外渗水检测（背景差分）、传感器轮询、显示、巡检保存全部独立线程/回调组；每级只保留最新帧，慢消费者丢旧帧不回压。
- 同屏面板每个相机窗格信息条实时显示两个帧率：`src=` 取原相机原图帧率（帧间到达间隔 EMA，验证取图是否正常）、`det=` 模型处理完画框后的输出帧率（解释窗口刷新快慢；推理慢时 src 不变、det 下降，一眼定位瓶颈在拿图还是在模型）。
- 面板为全屏窗口（`fullscreen`，默认开）：屏幕分辨率经 xrandr 自动探测（失败时用 `screen_width`/`screen_height` 参数兜底），并按显示器 DPI 换算 Qt 逻辑像素，保证高 DPI 屏幕也严格三等分、不裁切第三路；信息互不遮挡：窗格标题与帧率在顶部信息条、传感器块在第一路其下方、导航通信块在第二路其下方、总图右下角为显示刷新率。
- 传感器数据全部打印：T/RH/CO2 三值 + 温湿度/CO2 逐字段有效位（TH=ok/FAIL, CO2=ok/FAIL）+ 数据年龄 age；数据陈旧超过 3s 按离线显示。
- 导航协议实时块（第二个实时窗格顶部）：`NAV->VIS 0x.. VIS->NAV 0x..` + 状态词——`WAIT CMD`=等待指令、`CAPTURING`=取图中、`DONE (wait nav 0x00)`=取图完成等导航确认；颜色区分（绿=空闲/黄=取图中/橙=完成待确认）。
- 离线标识：core_node 状态行 `left_hz=0(OFF)`、`env[no_data]`；驱动日志 2~5s 节流告警。

## 模拟测试

```bash
bash tests/run_tests.sh          # 全部（含干净重建，约 8 分钟）
bash tests/run_tests.sh 05       # 只跑巡检协议
colcon test --packages-select culvert_inspection   # 状态机 gtest 单测
colcon test --packages-select tas_sensor_driver --ctest-args -R '^test_modbus_protocol$'
                                             # 厂商 V1.6 Modbus 报文/CRC 单测
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

传感器驱动另有 3 个协议单测：直接使用塔石《传感器寄存器定义说明 V1.6》的温湿度、CO2 请求与应答字节，校验 `9600 8N1` 默认协议中的寄存器、CRC 和 16 位数据解码；不需要连接串口设备。

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

0. **推荐：先跑配置向导**（见上文「新机开箱配置」；网卡名由 udev 生成、每台机器不同，不要照抄 `enx…`）。
1. （手动方式）相机 JSON 回填两台大恒 serial_number，`sim_mode` 置 false（或 `./run.sh sim:=false`）。
2. 红外 `config/infrared_camera.json` 填 device_path；本机热像为 `/dev/video0`（`/dev/video1` 多为 metadata，不是红外）。
3. 传感器 yaml 确认 ttyUSB 与 Modbus 地址。
4. 标定后回填三相机内参。
5. 巨帧：`packet_size` 8192 需主机 MTU≥9000；与 Mid360 共用交换机时计入上行带宽。

## 已知限制

- **真机验证状态（2026-09-14 USB3 扩展坞）**：
  - ✓ 双大恒：交换机经 `enx00e04c1e2b40`（RTL8152，USB 5000 Mbps / 千兆以太网）上联；向导双机实测 **12 fps** / 包长 8192 / MTU 9000，完整帧 `37/33`、残帧 0
  - ✓ 红外：UVC 候选在同坞 USB2（480 Mbps）分支；红外/传感器的低带宽 USB 支路不影响相机网卡的 USB3 带宽判定
  - ✓ 配置向导：识别 USB3 坞与板载直插两种路径、忽略无载波坞网口残留地址、正确接受 169.254 link-local 地址并保留 mid360 `192.168.1.50`
  - ✓ 取流：旧版手写 `GXDQBuf`/`GXQBuf` 声明错误会导致「恰好 5 张残帧然后断流」，已按官方签名修正
  - ○ 传感器：本次验证未接入；向导会按 `/dev/ttyUSB*`/`ttyACM*` 检测，未确认时不会误置真机模式
  - ⚠ YOLO：本机 OpenCV 4.5.4 无法加载当前 onnx，左右视觉降级透传
  - 注意：双 5MP 满帧约 20fps×2 超千兆，不要把 JSON 帧率抬到向导测稳值以上
- **conda/venv**：禁止在虚拟环境里 `colcon` / `./run.sh` / 测试；`env.sh` 会自动 deactivate
- 巡检"t 后第 2 帧"以巡检节点**到达顺序**为准；DDS 传输有毫秒级延迟，测试用静默间隔消除边界歧义。
- 保存中的 0x01 用 try_lock 拒绝（不排队）；周期结束后再来的 0x01 视为新一轮巡检。
- Release 构建类型未启用：-O3 会误报 galaxy SDK 封装的 stringop-overflow。
- core_node 同屏面板在无 DISPLAY 环境强制关闭（状态可经 `/core_node/status` 观测）。
