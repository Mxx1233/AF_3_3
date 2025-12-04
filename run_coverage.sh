#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: ./run_coverage.sh <package_name>"
    exit 1
fi

PKG_NAME=$1

echo "--- 1. Cleaning and Building $PKG_NAME with Coverage Flags ---"
colcon build --packages-select "$PKG_NAME" --symlink-install --cmake-clean-cache || exit 1
rm -rf log build/"$PKG_NAME"
mkdir -p build/"$PKG_NAME"

export COVERAGE_RUN=1
colcon build --packages-select "$PKG_NAME" --symlink-install \
  --cmake-args \
    -DCMAKE_CXX_FLAGS="-fprofile-arcs -ftest-coverage" \
    -DCMAKE_C_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1" \
  || exit 1

echo "--- 2. Initializing LCOV Counters ---"
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

RAW_INFO_FILE="lcov/total_coverage.info"
FINAL_INFO_FILE="lcov/${PKG_NAME}_final.info"

lcov --remove "$RAW_INFO_FILE" \
     "/opt/ros/*" "/usr/*" "*/test/*" "*/msg/detail/*" "*/src/*_node.cpp"\
     -o "$FINAL_INFO_FILE" \
     || { echo "LCOV remove failed!"; exit 1; }

genhtml "$FINAL_INFO_FILE" -o lcov/html || exit 1

echo "--- 6. Coverage Report Summary ---"
lcov --summary "$FINAL_INFO_FILE"


