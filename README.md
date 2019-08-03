# quadrupedal_gazebo_sim

### Description

A quadrupedal Gazebo simulation demo for 2019 ROBOCON

### Environment

Has been tested on ROS melodic full desktop

### Installation

```shell
git clone https://github.com/F-sf/quadrupedal_gazebo_sim.git ~/quadrupedal_gazebo_sim
cp -r ~/quadrupedal_gazebo_sim/maxliebao ~/catkin_ws/src/
echo export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/maxliebao/meshes >> ~/.bashrc
```

### Usage

```shell
roslaunch maxliebao gazebo.launch
roslaunch maxliebao control.launch
```





