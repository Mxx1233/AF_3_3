# this script must install all necessary components to build and test
# the code within this repository and that are not contained in the
# base docker image used by the CI
curl --header "JOB-TOKEN: $CI_JOB_TOKEN" "https://git-ce.rwth-aachen.de/api/v4/projects/1881/packages/generic/debian/2.3.3/ros-jazzy-psaf-ucbridge-msgs_2.3.3-0noble_amd64.deb" --output ros-jazzy-psaf-ucbridge-msgs_2.3.3-0noble_amd64.deb
curl --header "JOB-TOKEN: $CI_JOB_TOKEN" "https://git-ce.rwth-aachen.de/api/v4/projects/1881/packages/generic/debian/2.3.3/ros-jazzy-psaf-ucbridge_2.3.3-0noble_amd64.deb" --output ros-jazzy-psaf-ucbridge_2.3.3-0noble_amd64.deb
dpkg -i ros-jazzy-psaf-ucbridge-msgs_2.3.3-0noble_amd64.deb
dpkg -i ros-jazzy-psaf-ucbridge_2.3.3-0noble_amd64.deb
rm ros-jazzy-psaf-ucbridge-msgs_2.3.3-0noble_amd64.deb
rm ros-jazzy-psaf-ucbridge_2.3.3-0noble_amd64.deb

apt-get update && apt-get install -y ros-jazzy-foxglove-bridge
apt-get update && apt-get install -y \
    ros-jazzy-foxglove-bridge \
    python3-pip

pip install --upgrade pip setuptools
pip install build ultralytics --ignore-installed --break-system-packages
