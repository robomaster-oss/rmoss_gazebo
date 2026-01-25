#!/bin/bash
# Install necessary tools
apt-get update
apt-get install -y curl lsb-release gnupg

# Setup Gazebo Harmonic repository
curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# Update and install Gazebo Harmonic
apt-get update -qq
apt-get install -y gz-harmonic

# Install ROS 2 Gazebo bridge
# Note: ros-jazzy-ros-gz is the new package name replacing ros-jazzy-ros-ign
apt-get install -y// filepath: /home/xlqmu/rmoss_gazebo/.github/workflows/setup_gazebo.sh
#!/bin/bash
# Install necessary tools
apt-get update
apt-get install -y curl lsb-release gnupg

# Setup Gazebo Harmonic repository
curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# Update and install Gazebo Harmonic
apt-get update -qq
apt-get install -y gz-harmonic

# Install ROS 2 Gazebo bridge
# Note: ros-jazzy-ros-gz is the new package name replacing ros-jazzy-ros-ign
apt-get install -y ros-jazzy-ros-gz