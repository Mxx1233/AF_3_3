#!/bin/bash

# 检查是否提供了包名
if [ -z "$1" ]; then
    echo "Usage: ./run_coverage.sh <package_name>"
    exit 1
fi

PKG_NAME=$1

echo "--- 1. Cleaning and Building $PKG_NAME with Coverage Flags ---"
# 清理旧的构建和测试文件
colcon build --packages-select "$PKG_NAME" --symlink-install --cmake-clean-cache || exit 1
rm -rf log build/"$PKG_NAME"
mkdir -p build/"$PKG_NAME"

# 重新构建，插入覆盖率探针
export COVERAGE_RUN=1
colcon build --packages-select "$PKG_NAME" --symlink-install \
  --cmake-args \
    -DCMAKE_CXX_FLAGS="-fprofile-arcs -ftest-coverage" \
    -DCMAKE_C_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1" \
  || exit 1

echo "--- 2. Initializing LCOV Counters ---"
# 初始化 LCOV，注意排除系统路径和忽略错误
colcon lcov-result --initial --packages-select "$PKG_NAME" \
  --lcov-args \
    --ignore-errors mismatch \
  || exit 1

echo "--- 3. Running Tests (Generating .gcda files) ---"
colcon test --packages-select "$PKG_NAME" --executor sequential || exit 1

echo "--- 4. Generating Final LCOV Report ---"
colcon lcov-result --packages-select "$PKG_NAME" --verbose \
  --lcov-args \
    --ignore-errors mismatch \
  || exit 1


echo "--- 5. Cleaning and Finalizing Report ---"

# **重要步骤：手动运行 lcov --remove 来排除系统文件**
# colcon lcov-result 命令生成的报告在 lcov/lcov_all.info
# 找到生成的报告文件路径
RAW_INFO_FILE="lcov/total_coverage.info"
FINAL_INFO_FILE="lcov/${PKG_NAME}_final.info"

# 使用 --remove 来清除 /opt/ros/ 和 /usr/ 下的文件
lcov --remove "$RAW_INFO_FILE" \
     "/opt/ros/*" "/usr/*" "*/test/*" "*/msg/detail/*" "*/src/*_node.cpp"\
     -o "$FINAL_INFO_FILE" \
     || { echo "LCOV remove failed!"; exit 1; }

# 替换摘要文件
genhtml "$FINAL_INFO_FILE" -o lcov/html || exit 1

echo "--- 6. Coverage Report Summary ---"
# 这里我们现在看的是 genhtml 的结果，或者直接看 lcov -q summary
lcov --summary "$FINAL_INFO_FILE"


