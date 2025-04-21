#!/bin/bash

set -e

echo "Starting sim2real environment setup..."

# === 1. Create workspace ===
mkdir -p ~/sim2real/src
cd ~/sim2real

# === 2. Install base packages ===
echo "Installing base packages..."
sudo apt update
sudo apt install -y git curl wget net-tools netcat tmux htop bc

# === 3. Install PX4-Autopilot ===
if [ ! -d "$HOME/PX4-Autopilot" ]; then
  echo "Cloning PX4-Autopilot..."
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
  bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh
  cd ~/PX4-Autopilot && make px4_sitl
fi

# === 4. Install ROS 2 Humble ===
if ! source /opt/ros/humble/setup.bash 2>/dev/null; then
  echo "Installing ROS 2 Humble..."
  sudo apt install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  sudo apt install -y software-properties-common
  sudo add-apt-repository universe -y
  sudo apt update
  sudo apt install -y curl gnupg2 lsb-release

  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

  sudo apt update && sudo apt upgrade -y
  sudo apt install -y ros-humble-desktop ros-dev-tools
  source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
fi

# === 5. Install Python ROS tools ===
echo "Installing Python ROS packages..."
pip3 install --user --upgrade pip
pip3 install --user empy==3.3.4 pyros-genmsg setuptools

# === 6. Install Micro XRCE DDS Agent ===
if [ ! -d "$HOME/Micro-XRCE-DDS-Agent" ]; then
  echo "Installing Micro-XRCE-DDS-Agent..."
  git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent
  cd ~/Micro-XRCE-DDS-Agent
  sed -i 's/set(_fastdds_tag 2.12.x)/set(_fastdds_tag 2.14.4)/' CMakeLists.txt
  mkdir -p build && cd build
  cmake ..
  make -j$(nproc)
  sudo make install
  sudo ldconfig /usr/local/lib/
fi

# === 7. Clone PX4 ROS interfaces ===
cd ~/sim2real/src
[ ! -d "px4_msgs" ] && git clone https://github.com/PX4/px4_msgs.git
[ ! -d "px4_ros_com" ] && git clone https://github.com/PX4/px4_ros_com.git

# === 8. Build PX4 ROS interface ===
cd ~/sim2real
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash

# === 9. Install additional dependencies ===
echo "Setup complete. Please restart the terminal or run: source ~/.bashrc"
