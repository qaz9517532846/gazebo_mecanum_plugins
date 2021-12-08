
#ifndef DIFFDRIVE_PLUGIN_HH
#define DIFFDRIVE_PLUGIN_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_mecanum_plugins/gazebo_ros_mecanum_pid.h>
#include <gazebo_mecanum_plugins/gazebo_mecanum_plugins_vel.h>
#include <gazebo_mecanum_plugins/gazebo_mecanum_plugins_pid.h>

namespace gazebo
{
  class GazeboRosMecanumDrive : public ModelPlugin
  {
      public:
        GazeboRosMecanumDrive();
        ~GazeboRosMecanumDrive();
        void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

      protected:
        virtual void UpdateChild();
        virtual void FiniChild();

      private:
        void publishOdometry(double step_time);

        physics::ModelPtr parent_;
        event::ConnectionPtr update_connection_;

        boost::shared_ptr<ros::NodeHandle> rosnode_;
        ros::Publisher odometry_pub_;
        ros::Publisher joint_state_publisher_;
        sensor_msgs::JointState joint_state_;
        ros::Subscriber vel_sub_;
        boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
        nav_msgs::Odometry odom_;
        std::string tf_prefix_;

        boost::mutex lock;

        std::string robot_namespace_;
        std::string command_topic_;
        std::string odometry_topic_;
        std::string odometry_frame_;
        std::string robot_base_frame_;
        double odometry_rate_;

        // Custom Callback Queue
        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;
        void QueueThread();

        struct linear_vel
        {
          double vel_x;
          double vel_y;
          double vel_th;
        };

        linear_vel cal_LineVel_;

        // command velocity callback
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
        void calkinematics(linear_vel &line_vel);
        void getWheelVelocity();
        void publishWheelJointState();
        void publishWheelTF();

        double x_;
        double y_;
        double rot_;
        bool alive_;
        common::Time last_odom_publish_time_;
        ignition::math::Pose3d last_odom_pose_;

        // four wheel speed
        double wheel_speed_[4];
        double wheel_speed_instr_[4];

        // wheel joint name
        std::string left_rear_;
        std::string left_front_;
        std::string right_front_;
        std::string right_rear_;

        double wheel_separation_w;
        double wheel_separation_l;
        double wheel_torque;
        double wheel_accel_;
        double wheel_diameter_;

        // model wheel joint
         std::vector<physics::JointPtr> joints_;

        // Flags
        bool publishWheelTF_;
        bool publishWheelJointState_;
        bool publishOdomentry_;
        bool isRollerModel_;

        //gazebo plugin mecanum mobile robot with PID Controller.
        pid_controllerPtr pid_controller_;
        double dt_;

        std::string Wheel_I_vel;
        std::string Wheel_O_vel;
        std::string wheel_pid;

        ros::Publisher Wheel_I_vel_pub_;
        ros::Publisher Wheel_O_vel_pub_;

        gazebo_mecanum_plugins::gazebo_mecanum_plugins_vel wheel_input_;
        gazebo_mecanum_plugins::gazebo_mecanum_plugins_vel wheel_output_;

        ros::Subscriber wheel_pid_sub_;
        void wheelPIDCallback(const gazebo_mecanum_plugins::gazebo_mecanum_plugins_pid::ConstPtr& msg);
  };
}

#endif
