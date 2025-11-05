#!/usr/bin/env bash

CYAN='\033[0;36m'
RED='\033[0;31m'
NC=$'\033[0m' # No Color

echo -e "${CYAN}This script will calculate the coverage for one package${NC}"
echo -n "Enter package name: "
read -r package_name
dir="$(pwd)/src/$package_name"
if [ -d "$dir" ]; then
  echo -e "${CYAN}Found the package${NC}"
else
    echo -e "${RED}The package does not exist${NC}"
    exit 1
fi

cd src/$package_name
echo -e "${CYAN}Running uncrustify${NC}"
ament_uncrustify --reformat
cd ..
cd ..
echo -e "${CYAN}Building the project with cmake flags${NC}"
colcon build --packages-select psaf_configuration
colcon build --packages-select psaf_state_machine --cmake-args -DCMAKE_CXX_FLAGS="-fprofile-arcs -ftest-coverage " -DCMAKE_C_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1"
echo -e "${CYAN}Initializing lcov${NC}"

colcon lcov-result --packages-select "$package_name" --zero-counters
colcon lcov-result --packages-select "$package_name" --initial
echo -e "${CYAN}Running the tests${NC}"
colcon test --packages-select $package_name
echo -e "${CYAN}Generating the coverage report${NC}"
colcon lcov-result --packages-select "$package_name" --verbose
echo -e "${CYAN}Done...${NC}"
