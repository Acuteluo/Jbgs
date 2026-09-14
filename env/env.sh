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

# 禁止在 conda/venv 里构建或启动: ROS Humble 只用系统 python3.10。
# run.sh、tests/run_tests.sh、以及助手命令都必须走这一段。
_jbgs_deactivate_venv() {
    local _saved_u=0
    case "$-" in *u*) _saved_u=1; set +u ;; esac

    if [ -n "${CONDA_PREFIX:-}" ] || [ -n "${CONDA_DEFAULT_ENV:-}" ] ||
       { [ -n "${CONDA_SHLVL:-}" ] && [ "${CONDA_SHLVL}" != "0" ]; }; then
        echo "[env] 检测到 conda(${CONDA_DEFAULT_ENV:-unknown}), deactivate"
        if command -v conda >/dev/null 2>&1; then
            eval "$(conda shell.bash hook 2>/dev/null)" || true
            local n=0
            while [ -n "${CONDA_PREFIX:-}" ] && [ "$n" -lt 8 ]; do
                conda deactivate || break
                n=$((n + 1))
            done
        fi
        unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER \
            CONDA_PYTHON_EXE CONDA_SHLVL
    fi
    if [ -n "${VIRTUAL_ENV:-}" ]; then
        echo "[env] 检测到 venv(${VIRTUAL_ENV}), deactivate"
        if command -v deactivate >/dev/null 2>&1; then
            deactivate || true
        fi
        unset VIRTUAL_ENV
    fi
    if [ -x /usr/bin/python3 ]; then
        PATH="/usr/bin:/usr/local/bin:${PATH}"
        export PATH
        hash -r 2>/dev/null || true
        export Python3_EXECUTABLE=/usr/bin/python3
    fi
    if [ "$_saved_u" -eq 1 ]; then set -u; fi
}
_jbgs_deactivate_venv
unset -f _jbgs_deactivate_venv

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
