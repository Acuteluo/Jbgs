# AGENTS.md — Jbgs 工程约定（给后续维护者/Agent）

## 工程事实

- 工作目录：仓库根（`run.sh` 所在目录；脚本一律从自身路径定位，禁止写死用户目录）。
- 唯一入口：`./run.sh`（干净重建 + 启动）；`./run.sh --build-only` 只构建。
- 构建：`colcon build --symlink-install`，默认构建类型（勿开 Release，见 README 已知限制）。
- 启动：`ros2 launch bringup all.launch.py`，配置装载见 `src/bringup/launch/launch_common.py`（launch.json → 相机 JSON → yaml → launch 参数，优先级从低到高）。

## 硬性规则

1. **相对路径**一律经 `JBGS_ROOT` 环境变量解析（run.sh/env.sh 导出；C++ 侧 `ResolvePath()` 已有实现：core_node / inspection_node / galaxy_camera_node）。
2. **巡检协议**（`culvert_inspection`，导航↔视觉：下行 `/vision_capture_cmd`、上行 `/vision_capture_status`，UInt8 电平 20Hz；完成后门控等 0x00）状态机逻辑只改 `inspection_fsm.hpp`，改完必须跑：
   - `colcon test --packages-select culvert_inspection`（gtest 单测，13 个用例）
   - `bash tests/run_tests.sh 05`（端到端）
   - 不变量：双帧齐才写盘；写盘全成才进入 0x02 完成态（持续广播至收到 0x00）；单侧失败清理半对文件；处理中 0x01 被忽略；**0x00 必须先于"非 0x01 拒绝"处理**（解除完成门控，放后面就是死代码，第二站触发会被永久忽略——实车出过事故，case E 回归）。
   - 新鲜度校验在 core_node 面板（`nav_cmd_fresh_sec` / `capture_done_timeout_sec`，launch.json 可调）：电平为 0x00 区分不出"在行车"与"导航没接入"，必须看消息到达时刻。
3. **测试全部在 `tests/`**（C++ + ctest）与包内 gtest；不新增 pytest 除非团队一致同意；不依赖真机与外部数据集。
4. **模拟语义**：sim 设备与真机走同一套"超时判定 → 重连"路径；改驱动热插拔逻辑时保证模拟分支不旁路。
5. `models/`、`pictures/` 是实拍资源，**不得删除或移动**。
6. `Culvert-Visual-Inspection-main/` 是红外参考工程（有 COLCON_IGNORE），不能参与构建；红外算法已实现于 `culvert_core/ir_seepage_detector.*`。
7. 根目录只保留 `AGENTS.md`、`README.md`、`run.sh`、`easy_setting.md` 四个文件，其余入目录。
8. 中文注释：包/文件头写明职责；关键键位、开关、状态机分支必须有注释；异常路径要有详细报错。

## 测试

```bash
bash tests/run_tests.sh                              # 全量（约 8 分钟）
bash tests/run_tests.sh 05                           # 巡检协议相关
colcon test --packages-select culvert_inspection     # 状态机单测
```

- 测试隔离：`ROS_DOMAIN_ID=93`（ctest ENVIRONMENT 注入），避免与本机其他 ROS 会话互扰。
- `ros2` CLI 在交互 shell 间歇崩溃：验证类操作放进脚本/测试节点执行，不要手工敲。

## 已知坑

- **禁止在 conda/venv 里构建、启动、测试**（含 Cursor 助手）：ROS Humble 用系统 python3.10，conda python 会让 colcon 找不到 `catkin_pkg`/`em`。`env.sh` 会自动 `conda deactivate`；命令前仍应先退环境。
- `set -u` 与 ROS setup.bash 冲突：source 前后用 `set +u / set -u` 包裹（run.sh、env.sh、测试脚本均已处理）。
- launch 框架的 `launch_arguments` 必须传 `[(name, value)]` 元组列表，不能传 dict。
- pkill -f 会匹配到执行它的命令块自身：清理进程用括号技巧（`nod[e]`）或精确可执行名。
- `/dev/video0、/dev/video1` 是本机真摄像头，不是红外：红外模拟测试勿用 device_index 0/1 直开。
