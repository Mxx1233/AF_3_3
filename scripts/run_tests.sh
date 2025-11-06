#!/usr/bin/env bash

CYAN='\033[0;36m'
RED='\033[0;31m'
NC=$'\033[0m' # No Color

echo -e "${CYAN}This script will build one package and run the tests${NC}"
echo -n "Enter package name: "
read package_name
dir="$(pwd)/src/$package_name"
if [ -d "$dir" ]; then
  echo -e "${CYAN}Found the package${NC}"
else
    echo -e "${RED}The package does not exist${NC}"

fi

cd src/$package_name
echo -e "${CYAN}Running uncrustify${NC}"
ament_uncrustify --reformat
cd ..
cd ..
echo -e "${CYAN}Building the package...${NC}"
colcon build --packages-select psaf_configuration
colcon build --packages-select $package_name --symlink-install
echo -e "${CYAN}Running tests...${NC}"
colcon test --packages-select $package_name --event-handlers console_direct+
echo -e "${CYAN}Done Testing, Please check log if all tests succeeded${NC}"



