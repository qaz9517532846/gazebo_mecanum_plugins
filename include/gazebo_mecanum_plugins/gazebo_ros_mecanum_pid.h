#ifndef DIFFDRIVE_PID_HH
#define DIFFDRIVE_PID_HH

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <boost/bind.hpp>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>

namespace gazebo
{
    class pid_controller
    {
        public:
          pid_controller(physics::ModelPtr model);
          ~pid_controller();

          void gazebo_PID_LEFT_REAR_(physics::JointPtr joint, double vel);
          void gazebo_PID_LEFT_FRONT_(physics::JointPtr joint, double vel);
          void gazebo_PID_RIGHT_FRONT_(physics::JointPtr joint, double vel);
          void gazebo_PID_RIGHT_REAR_(physics::JointPtr joint, double vel);

          struct PID_Parameter
          {
            double P;
            double I;
            double D;
          };

          PID_Parameter LEFT_REAR_, LEFT_FRONT_, RIGHT_FRONT_, RIGHT_REAR_;

        private:
          double max_value_;
          double min_value_;

          physics::ModelPtr mobile_model;

          common::PID LEFT_REAR_pid;
          common::PID LEFT_FRONT_pid;
          common::PID RIGHT_FRONT_pid;
          common::PID RIGHT_REAR_pid;
    };
    typedef boost::shared_ptr<pid_controller> pid_controllerPtr;
}

#endif