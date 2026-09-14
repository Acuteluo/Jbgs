#!/usr/bin/env bash
# ============================================================
# 文件: tests/run_tests.sh
# 作用: 一键运行全部自动化测试(工程根 tests/ 目录, ctest 编排)。
#
# 用法:
#   bash tests/run_tests.sh            # 全部测试(含干净重建, 约 8 分钟)
#   bash tests/run_tests.sh 05         # 只跑名字含 05 的测试
#
# 流程:
#   1. 校验 ROS2 环境; install 缺失时先执行一次 ./run.sh --build-only;
#   2. 配置并构建 tests/ 下的 C++ 系统测试可执行文件;
#   3. ctest 串行执行(01_clean_build 会再做一次可控干净重建);
#   4. 输出每项测试结果汇总。
#
# 说明: 单元测试(巡检状态机 gtest)由 colcon 侧执行:
#   colcon test --packages-select culvert_inspection
# 本脚本末尾会自动带上这一步。
# ============================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$TESTS_DIR/.." && pwd)"
export JBGS_ROOT="$PROJECT_ROOT"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-93}"

FILTER="${1:-}"

echo "[run_tests] 工程根: $PROJECT_ROOT"

# ---- 1. ROS2 环境(env.sh 会 conda deactivate, 禁止在虚拟环境里测) ----
# shellcheck disable=SC1091
set +u
source "$PROJECT_ROOT/env/env.sh" || {
    echo "[run_tests] 错误: 环境装载失败" >&2
    exit 1
}
set -u

# install 缺失时先引导构建(ctest 的 01_clean_build 仍会做正式干净重建)
if [ ! -f "$PROJECT_ROOT/install/setup.bash" ]; then
    echo "[run_tests] install 缺失, 先执行引导构建 ..."
    bash "$PROJECT_ROOT/run.sh" --build-only || exit 1
fi
set +u
source "$PROJECT_ROOT/install/setup.bash"
set -u

# ---- 1.5 清扫历史残留进程(孤儿节点会让所有"无残留"断言失败) ----
for pat in "galaxy_camera_dual_nod[e]" "core_nod[e]" "inspection_nod[e]" \
           "ir_camera_driver_nod[e]" "tas_sensor_driver_nod[e]" "ros2 launch bringu[p]"; do
    pkill -KILL -f "$pat" 2>/dev/null
done
sleep 1

# ---- 2. 构建系统测试可执行文件 ----
BUILD_DIR="$TESTS_DIR/build"
cmake -S "$TESTS_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=None > /tmp/jbgs_tests_cmake.log 2>&1 \
    || { echo "[run_tests] cmake 失败, 见 /tmp/jbgs_tests_cmake.log"; tail -20 /tmp/jbgs_tests_cmake.log; exit 1; }
cmake --build "$BUILD_DIR" -j"$(nproc)" > /tmp/jbgs_tests_make.log 2>&1 \
    || { echo "[run_tests] 构建失败, 见 /tmp/jbgs_tests_make.log"; tail -30 /tmp/jbgs_tests_make.log; exit 1; }

# ---- 3. ctest 串行执行 ----
cd "$BUILD_DIR"
ctest_ARGS=(--output-on-failure)
if [ -n "$FILTER" ]; then
    ctest_ARGS+=(-R "$FILTER")
fi
ctest "${ctest_ARGS[@]}"
CTEST_RC=$?

# ---- 4. 单元测试(colcon 侧 gtest) ----
echo ""
echo "[run_tests] 单元测试: colcon test --packages-select culvert_inspection"
cd "$PROJECT_ROOT"
colcon test --packages-select culvert_inspection > /tmp/jbgs_unit_test.log 2>&1
UNIT_RC=$?
colcon test-result --verbose 2>/dev/null | tail -5 || true

echo ""
if [ "$CTEST_RC" -eq 0 ] && [ "$UNIT_RC" -eq 0 ]; then
    echo "[run_tests] 全部测试通过 ✓"
    exit 0
fi
echo "[run_tests] 存在失败: ctest=$CTEST_RC colcon_test=$UNIT_RC"
exit 1
