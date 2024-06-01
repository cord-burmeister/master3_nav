# Navigation Playground

>**Note:** This repository is under construction. It has no working functionality yet.

## Overview

This repository provides a robot description and a simulation environment together with navigation stack as playground for robotics experiments.

## Prerequisites

* Installed ROS 2 humble distribution
* Installed gazebo harmonic distribution
* Setting *export GZ_VERSION=harmonic* (best in bashrc) to define target gazebo version

## Installation

First install required development tools

``` bash
sudo apt install python3-vcstool python3-colcon-common-extensions git wget
```

Then create a new workspace and load the git repositories which are required.

``` bash
mkdir -p ~/master3_ws/src
cd ~/master3_ws/src
wget https://raw.githubusercontent.com/cord-burmeister/master3_nav/main/master3.yaml
vcs import < master3.yaml
```

### Install dependencies

``` bash
cd ~/master3_ws
source /opt/ros/$ROS_DISTRO/setup.bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro $ROS_DISTRO
```

### Build the project

``` bash
colcon build 
```

### Source the workspace

``` bash
. ~/master3_ws/install/setup.sh
```

## Starting commands

### Starting the simulation environment

This opens the *gazebo* together with the *RViz2* with the spawned robot.

``` bash
ros2 launch master3_bringup master3_house.launch.py
```
