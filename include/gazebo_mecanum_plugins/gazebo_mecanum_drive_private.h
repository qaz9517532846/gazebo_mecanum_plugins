#ifndef GAZEBO_MECANUM_PLUGINS_DIFF_DRIVE_PRIVATE_H_
#define GAZEBO_MECANUM_PLUGINS_DIFF_DRIVE_PRIVATE_H_

#include <gazebo/common/Time.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>

#ifdef NO_ERROR
// NO_ERROR is a macro defined in Windows that's used as an enum in tf2
#undef NO_ERROR
#endif

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_mecanum_plugins
{
  class GazeboMecanumRosDrivePrivate
  {
    public:
  
    /// Indicates which wheel
    enum
    {
      LEFT_REAR = 0,
      LEFT_FRONT = 1,
      RIGHT_FRONT = 2,
      RIGHT_REAR = 3,
    };
    
    /// Callback to be called at every simulation iteration.
    /// \param[in] _info Updated simulation info.
    void OnUpdate(const gazebo::common::UpdateInfo & _info);

    /// Callback when a velocity command is received.
    /// \param[in] _msg Twist command message.
    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

    /// Update wheel velocities according to latest target velocities.
    void UpdateWheelVelocities();

    /// Publish odometry transforms
    /// \param[in] _current_time Current simulation time
    void PublishOdometryTf(const gazebo::common::Time & _current_time);

    /// Publish trasforms for the wheels
    /// \param[in] _current_time Current simulation time
    void PublishWheelsTf(const gazebo::common::Time & _current_time);

    /// Publish odometry messages
    /// \param[in] _current_time Current simulation time
    void PublishOdometryMsg(const gazebo::common::Time & _current_time);

    /// A pointer to the GazeboROS node.
    gazebo_ros::Node::SharedPtr ros_node_;

    /// Subscriber to command velocities
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    /// Odometry publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

    /// Connection to event called at every world iteration.
    gazebo::event::ConnectionPtr update_connection_;

    /// Distance between the wheels, in meters.
    std::vector<double> wheel_w_separation_;
    std::vector<double> wheel_l_separation_;

    /// Diameter of wheels, in meters.
    std::vector<double> wheel_diameter_;

    /// Maximum wheel torque, in Nm.
    double max_wheel_torque_;

    /// Maximum wheel acceleration
    double max_wheel_accel_;

    /// Desired wheel speed.
    std::vector<double> desired_wheel_speed_;

    /// Speed sent to wheel.
    std::vector<double> wheel_speed_instr_;

    /// Pointers to wheel joints.
    std::vector<gazebo::physics::JointPtr> joints_;

    /// Pointer to model.
    gazebo::physics::ModelPtr model_;

    /// To broadcast TFs
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

    /// Protect variables accessed on callbacks.
    std::mutex lock_;

    /// Linear velocity in X received on command (m/s).
    double target_x_{0.0};

    /// Linear velocity in Y received on command (m/s).
    double target_y_{0.0};

    /// Angular velocity in Z received on command (rad/s).
    double target_rot_{0.0};

    /// Update period in seconds.
    double update_period_;

    /// Last update time.
    gazebo::common::Time last_update_time_;

    /// Keep encoder data.
    geometry_msgs::msg::Pose2D pose_encoder_;

    /// Odometry frame ID
    std::string odometry_frame_;

    /// Odometry topic
    std::string odometry_topic_;

    /// cmd topic
    std::string cmd_topic_;

    /// Last time the encoder was updated
    gazebo::common::Time last_encoder_update_;

    /// Keep latest odometry message
    nav_msgs::msg::Odometry odom_;

    /// Robot base frame ID
    std::string robot_base_frame_;

    /// True to publish odometry messages.
    bool publish_odom_;

    /// True to publish wheel-to-base transforms.
    bool publish_wheel_tf_;

    /// True to publish odom-to-world transforms.
    bool publish_odom_tf_;

    /// Store number of wheel pairs
    unsigned int num_wheel_pairs_;

    /// Covariance in odometry
    double covariance_[3];

    /// Has the agv model roller model. 
    bool isRollerModel_;

    /// Get linear velocity after calculating mecanum kinematics. 
    struct linear_vel
    {
      double vel_x;
      double vel_y;
      double vel_th;
    };

    linear_vel cal_LineVel_;

    struct pid_parameter
    {
      double p_gain;
      double i_gain;
      double d_gain;
    };

    pid_parameter pid_lr, pid_lf, pid_rf, pid_rr;

    /// Calcalate mecanum wheel kinematics
    void calkinematics(linear_vel &line_vel);
    void gazebo_PID_WHEEL_(gazebo::physics::JointPtr joint, pid_parameter &pid, double vel);
  };
}  // namespace gazebo_plugins

#endif