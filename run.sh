#!/usr/bin/env bash
# ============================================================
# 文件: run.sh
# 作用: 一键入口 —— 定位工程根目录 -> 可控干净重建 -> source 环境
#       -> 启动总 launch(双大恒 + 红外 + 传感器 + core_node + 巡检协议)。
#
# 用法:
#   ./run.sh                    # 干净重建 + 启动全部(Ctrl+C 一次停全部)
#   ./run.sh --build-only       # 只做干净重建, 不启动(测试/CI 用)
#   ./run.sh sim:=false         # 真机模式启动(参数原样透传给 launch)
#   ./run.sh show_windows:=false in_trulyworking:=false
#
# 说明:
#   - 不依赖任何写死的用户目录; 工程根 = 本脚本所在目录;
#   - 节点内相对路径(模型/保存目录等)以 JBGS_ROOT 环境变量解析;
#   - "可控干净重建" = 仅删除本工程的 build/install/log 三个产物目录,
#     不触碰 models/pictures/config/src 等源与资源。
# ============================================================
set -euo pipefail

# ---- 1. 定位工程根目录(从自身路径, 兼容 bash/source 两种调用) ----
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export JBGS_ROOT="$PROJECT_ROOT"
cd "$PROJECT_ROOT"

# ---- 2. 解析本脚本自己的开关(其余参数原样透传给 launch) ----
BUILD_ONLY=0
LAUNCH_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --build-only|-b) BUILD_ONLY=1 ;;
        -h|--help)
            sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) LAUNCH_ARGS+=("$arg") ;;
    esac
done

# ---- 3. 可控干净重建: 删除旧产物目录(源码/模型/图片/配置不动) ----
echo "[run] 干净重建: 删除 build/ install/ log/ ..."
rm -rf "$PROJECT_ROOT/build" "$PROJECT_ROOT/install" "$PROJECT_ROOT/log"

# ROS2 环境(缺失时 env.sh 会明确报错并退出)
# shellcheck disable=SC1091
source "$PROJECT_ROOT/env/env.sh" || {
    echo "[run] 错误: 环境装载失败, 无法构建" >&2; exit 1; }

echo "[run] colcon build --symlink-install ..."
# 构建类型保持默认(与历史一致): Release 的 -O3 会把 SDK 封装里
# 一处 stringop-overflow 误报升级为错误, 导致整包构建失败。
colcon build --symlink-install --event-handlers console_direct+

echo "[run] 构建完成"

# ---- 4. --build-only: 到此为止 ----
if [ "$BUILD_ONLY" -eq 1 ]; then
    echo "[run] --build-only 指定, 不启动系统"
    exit 0
fi

# ---- 5. 重新 source(刚重建的 install)并启动总 launch ----
# shellcheck disable=SC1091
# colcon 生成的 setup.bash 直接引用 $COLCON_TRACE；在本脚本的 `set -u`
# 下，未设置该变量会被 Bash 当作错误。与 env/env.sh 一致，仅在 source
# 期间关闭 nounset，随后立即恢复严格模式。
set +u
source "$PROJECT_ROOT/install/setup.bash"
set -u

# 退出时确保 launch 进程组整体终止(无残留节点/子进程)
LAUNCH_PID=""
cleanup() {
    if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo ""
        echo "[run] 停止 launch 进程树 ..."
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        for _ in $(seq 1 50); do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 0.1
        done
        kill -TERM "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "[run] 启动: ros2 launch bringup all.launch.py ${LAUNCH_ARGS[*]:-}"
ros2 launch bringup all.launch.py "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!
wait "$LAUNCH_PID"
