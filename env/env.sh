#!/usr/bin/env bash
# ============================================================
# 文件: env/env.sh
# 作用: 环境装载脚本(run.sh 与测试共同使用)。
#   - source ROS2 Humble 与本工程 install 环境;
#   - 导出 JBGS_ROOT(工程根目录, 节点内相对路径以此解析);
#   - 不做任何构建/启动动作。
# 用法: source "$(dirname "${BASH_SOURCE[0]}")/../env/env.sh"
# ============================================================

# 工程根目录: 以本文件位置定位(env/ 的上一级), 与调用者 cwd 无关
JBGS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export JBGS_ROOT

# ROS2 Humble(缺失时明确报错, 不静默继续)
# 注意: ROS 的 setup.bash 含未绑定变量, 与 set -u 不兼容, source 期间临时关闭
if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
else
    echo "[env] 错误: 未找到 /opt/ros/humble/setup.bash, 请先安装 ROS2 Humble" >&2
    return 1 2>/dev/null || exit 1
fi

# 本工程 install 环境(尚未构建时给出提示但不报错, 便于首次构建前 source)
if [ -f "$JBGS_ROOT/install/setup.bash" ]; then
    set +u
    source "$JBGS_ROOT/install/setup.bash"
    set -u
else
    echo "[env] 提示: $JBGS_ROOT/install 尚不存在, 请先运行 ./run.sh --build-only" >&2
fi
