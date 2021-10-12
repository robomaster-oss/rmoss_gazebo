#!/bin/bash
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-prerelease.list
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list
wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
apt-get update -qq
apt-get install -y libignition-msgs7-dev libignition-transport10-dev libignition-gazebo5-dev