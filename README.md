# gazebo_mecanum_plugins

gazebo_mecanum_plugins is Gazebo plugins for mecanum driving.

------

## Built with

- ROS Foxy under Ubuntu 20.04 LTS

------

## Getting Started

### Installation

``` $ sudo apt-get install ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-ros-control ros-foxy-ros-controllers```
    
``` $ sudo apt-get install -y libgazebo11-dev```
    
``` $ sudo apt-get install -y gazebo11```
    
### How to using a gazebo_mecanum_plugins into xacro file.

``` bash
<gazebo>
      <plugin name="gazebo_ros_mecanum_diff_drive" filename="libgazebo_ros_mecanum_diff_drive.so">
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
    <plugin name="zm_robot_diff_drive" filename="libgazebo_mecanum_pid_drive.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>base_footprint</robotBaseFrame>
      <LeftRear>wheel_joint_1</LeftRear>
      <LeftFront>wheel_joint_2</LeftFront>
      <RightFront>wheel_joint_3</RightFront>
      <RightRear>wheel_joint_4</RightRear>
      <odometryRate>20</odometryRate>      
      <publishWheelTF>false</publishWheelTF>
      <publishOdomTF>true</publishOdomTF>
      <WheelTorque>30</WheelTorque>
      <WheelSeparationW>0.5</WheelSeparationW>
      <WheelSeparationL>0.6</WheelSeparationL>
      <wheelAccel>10</wheelAccel>
      <wheelDiameter>0.13</wheelDiameter>
      <publishOdom>true</publishOdom>
      <isRollerModel>true</isRollerModel>
      <!--PID Controller Parameter-->
      <Wheel_LR_P>10.0</Wheel_LR_P>
      <Wheel_LR_I>0.0</Wheel_LR_I>
      <Wheel_LR_D>0.0</Wheel_LR_D>
      <Wheel_LF_P>10.0</Wheel_LF_P>
      <Wheel_LF_I>0.0</Wheel_LF_I>
      <Wheel_LF_D>0.0</Wheel_LF_D>
      <Wheel_RF_P>10.0</Wheel_RF_P>
      <Wheel_RF_I>0.0</Wheel_RF_I>
      <Wheel_RF_D>0.0</Wheel_RF_D>
      <Wheel_RR_P>10.0</Wheel_RR_P>
      <Wheel_RR_I>0.0</Wheel_RR_I>
      <Wheel_RR_D>0.0</Wheel_RR_D>
    </plugin>
</gazebo>
```

------

### Reference

[1]. gazebo_ros_pkgs, https://github.com/ros-simulation/gazebo_ros_pkgs

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.
