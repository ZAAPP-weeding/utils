# Installation script for MRSD Team F24 - ZAAPP

set -e -x

WORKSPACE="~/zaapp_per_user/$USER"
ROS_DISTRO="humble" 

mkdir -p $WORKSPACE
cd $WORKSPACE

# Will fail if ROS is not installed. Installing ros is not included in this script.
source /opt/ros/$ROS_DISTRO/setup.bash

sudo apt update && sudo apt upgrade -y
sudo apt install python3-colcon-common-extensions python3-rosdep ros-$ROS_DISTRO-common-msgs -y

python3.12 -m venv venv
source venv/bin/activate
pip install numpy torch torchvision opencv-python scipy tqdm ultralytics

git clone https://github.com/ZAAPP-weeding/zaapp

colcon build

set +e +x 
echo ""
echo "Installation complete. To launch, run \`cd $WORKSPACE && bash zaapp/scripts/launch.bash\`"