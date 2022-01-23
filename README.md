# gazebo_mecanum_plugins

gazebo_mecanum_plugins is Gazebo plugins for mecanum driving.

------

## Built with

- ROS Noetic under Ubuntu 20.04 LTS

------

## Getting Started

### Installation

``` $ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-ros-controllers```
    
``` $ sudo apt-get install -y libgazebo11-dev```
    
``` $ sudo apt-get install -y gazebo11```
    
### How to using a gazebo_mecanum_plugins into xacro file.

``` bash
<gazebo>
      <plugin name="gazebo_ros_mecanum_diff_drive" filename="libgazebo_ros_mecanum_drive.so">
        <commandTopic>cmd_vel</commandTopic>
        <odometryTopic>odom</odometryTopic>
        <odometryFrame>odom</odometryFrame>
        <robotBaseFrame>base_footprint</robotBaseFrame>
        <LeftRear>wheel_joint_1</LeftRear>
        <LeftFront>wheel_joint_2</LeftFront>
        <RightFront>wheel_joint_3</RightFront>
        <RightRear>wheel_joint_4</RightRear>
        <odometryRate>20</odometryRate>      
        <publishWheelTF>true</publishWheelTF>
        <publishWheelJointState>true</publishWheelJointState>
        <WheelTorque>1</WheelTorque>
        <WheelSeparationW>0.5</WheelSeparationW>
        <WheelSeparationL>0.6</WheelSeparationL>
        <wheelAccel>5</wheelAccel>
        <wheelDiameter>0.13</wheelDiameter>
        <publishOdom>false</publishOdom>
        <isRollerModel>true</isRollerModel>
      </plugin>
</gazebo>
```
------

### Using gazebo mecanum plugins PID drive

``` bash
<gazebo>
      <plugin name="gazebo_mecanum_pid_plugin" filename="libgazebo_ros_mecanum_pid_drive.so">
        <commandTopic>cmd_vel</commandTopic>
        <odometryTopic>odom</odometryTopic>
        <odometryFrame>odom</odometryFrame>
        <robotBaseFrame>base_footprint</robotBaseFrame>
        <LeftRear>wheel_joint_1</LeftRear>
        <LeftFront>wheel_joint_2</LeftFront>
        <RightFront>wheel_joint_3</RightFront>
        <RightRear>wheel_joint_4</RightRear>
        <odometryRate>20</odometryRate>      
        <publishWheelTF>true</publishWheelTF>
        <publishWheelJointState>true</publishWheelJointState>
        <WheelTorque>100</WheelTorque>
        <WheelSeparationW>0.5</WheelSeparationW>
        <WheelSeparationL>0.6</WheelSeparationL>
        <wheelAccel>5</wheelAccel>
        <wheelDiameter>0.13</wheelDiameter>
        <publishOdom>false</publishOdom>
        <isRollerModel>true</isRollerModel>
        <dt>0.05</dt>
        <WheelInputVelTopic>zm_input_vel</WheelInputVelTopic>
        <WheelOutputVelTopic>zm_output_vel</WheelOutputVelTopic>
        <WheelPID>gazebo_mecanum_plugins_PID</WheelPID>
      </plugin>
</gazebo>
```

Launch PID Parameter.

``` bash
$ roslaunch gazebo_mecanum_plugins gazebo_mecanum_pid.launch
```

Ajust PID parameter.

``` bash
$ rosrun rqt_gui rqt_gui
```

------

### Reference

[1]. gazebo_ros_pkgs, https://github.com/ros-simulation/gazebo_ros_pkgs

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.
