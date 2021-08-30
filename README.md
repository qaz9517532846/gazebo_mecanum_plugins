# gazebo_mecanum_plugins

Gazebo Plugin for the mobile robot using four mecanum wheel. 

- Software: Robot Operating System 2.

- Version: foxy.

-  Step1. Install package.

- Your PC need to install ros package.

  ``` $ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-ros-controllers ```

  ``` $ sudo apt-get install -y libgazebo11-dev ```
    
  ``` $ sudo apt-get install -y gazebo11 ```

    
- Step2. Install gazebo_mecanum_plugins package.

``` bash
$ cd <catkin_workspace>/src
```

``` bash
$ git clone https://github.com/qaz9517532846/gazebo_mecanum_plugins.git
```

``` bash
$ cd ..
```

``` bash
$ catkin_make
```

- Step3. How to using a gazebo_mecanum_plugins into xacro file.

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

### Reference

[1]. gazebo_ros_pkgs, https://github.com/ros-simulation/gazebo_ros_pkgs

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.
