#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <gazebo_mecanum_plugins/gazebo_mecanum_plugins_pidConfig.h>
#include <gazebo_mecanum_plugins/gazebo_mecanum_plugins_pid.h>

double wheel1_p, wheel1_i, wheel1_d;
double wheel2_p, wheel2_i, wheel2_d;
double wheel3_p, wheel3_i, wheel3_d;
double wheel4_p, wheel4_i, wheel4_d;

bool isAjustPID_;

void pid_callback(gazebo_mecanum_plugins::gazebo_mecanum_plugins_pidConfig &config, uint32_t level) 
{
    wheel1_p = config.LEFT_REAR_P;
    wheel1_i = config.LEFT_REAR_I; 
    wheel1_d = config.LEFT_REAR_D;
    wheel2_p = config.LEFT_FRONT_P; 
    wheel2_i = config.LEFT_FRONT_I; 
    wheel2_d = config.LEFT_FRONT_D;
    wheel3_p = config.RIGHT_FRONT_P; 
    wheel3_i = config.RIGHT_FRONT_I; 
    wheel3_d = config.RIGHT_FRONT_D;
    wheel4_p = config.RIGHT_REAR_P; 
    wheel4_i = config.RIGHT_REAR_I; 
    wheel4_d = config.RIGHT_REAR_D;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "gazebo_mecanum_plugin_pid");
  ros::NodeHandle n;

  ros::NodeHandle prinate_nh("~");
  prinate_nh.param<double>("LRWheel_P", wheel1_p, 10.0);
  prinate_nh.param<double>("LRWheel_I", wheel1_i, 0.0);
  prinate_nh.param<double>("LRWheel_D", wheel1_d, 0.0);

  prinate_nh.param<double>("LFWheel_P", wheel2_p, 10.0);
  prinate_nh.param<double>("LFWheel_I", wheel2_i, 0.0);
  prinate_nh.param<double>("LFWheel_D", wheel2_d, 0.0);

  prinate_nh.param<double>("RFWheel_P", wheel3_p, 10.0);
  prinate_nh.param<double>("RFWheel_I", wheel3_i, 0.0);
  prinate_nh.param<double>("RFWheel_D", wheel3_d, 0.0);

  prinate_nh.param<double>("RRWheel_P", wheel4_p, 10.0);
  prinate_nh.param<double>("RRWheel_I", wheel4_i, 0.0);
  prinate_nh.param<double>("RRWheel_D", wheel4_d, 0.0);
  prinate_nh.param<bool>("isAjustPID", isAjustPID_, false);

  ROS_INFO("Set to ajust PID Parameter = %d", isAjustPID_);

  dynamic_reconfigure::Server<gazebo_mecanum_plugins::gazebo_mecanum_plugins_pidConfig> pid_server;
  dynamic_reconfigure::Server<gazebo_mecanum_plugins::gazebo_mecanum_plugins_pidConfig>::CallbackType f;
  
  ros::Publisher gazebo_ros_mecanum_pid = n.advertise<gazebo_mecanum_plugins::gazebo_mecanum_plugins_pid>("gazebo_mecanum_plugins_PID", 50);
  f = boost::bind(pid_callback, _1, _2);

  if(isAjustPID_)
  {
    pid_server.setCallback(f);
  }

  ros::Rate loop_rate(20);

  while(n.ok())
  {
    ros::spinOnce();
    gazebo_mecanum_plugins::gazebo_mecanum_plugins_pid wheel_PID_param;

    wheel_PID_param.LR_P = wheel1_p;
    wheel_PID_param.LR_I = wheel1_i;
    wheel_PID_param.LR_D = wheel1_d;

    wheel_PID_param.LF_P = wheel2_p;
    wheel_PID_param.LF_I = wheel2_i;
    wheel_PID_param.LF_D = wheel2_d;

    wheel_PID_param.RF_P = wheel3_p;
    wheel_PID_param.RF_I = wheel3_i;
    wheel_PID_param.RF_D = wheel3_d;

    wheel_PID_param.RR_P = wheel4_p;
    wheel_PID_param.RR_I = wheel4_i;
    wheel_PID_param.RR_D = wheel4_d;

    gazebo_ros_mecanum_pid.publish(wheel_PID_param);

    loop_rate.sleep();
  }

  return 0;
}