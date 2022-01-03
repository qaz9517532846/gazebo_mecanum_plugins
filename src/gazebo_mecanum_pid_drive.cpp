#include <gazebo_mecanum_plugins/gazebo_mecanum_drive.h>

namespace gazebo_mecanum_plugins
{
  GazeboMecanumRosDrive::GazeboMecanumRosDrive() : impl_(std::make_unique<GazeboMecanumRosDrivePrivate>())
  {

  }
  
  GazeboMecanumRosDrive::~GazeboMecanumRosDrive()
  {

  }
  
  void GazeboMecanumRosDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    impl_->model_ = _model;
    
    // Initialize ROS node
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Get QoS profiles
    const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

    // Get number of wheel pairs in the model
    impl_->num_wheel_pairs_ = static_cast<unsigned int>(_sdf->Get<int>("num_wheel_pairs", 1).first);

    if (impl_->num_wheel_pairs_ < 1)
    {
      impl_->num_wheel_pairs_ = 1;
      RCLCPP_WARN(impl_->ros_node_->get_logger(), "Drive requires at least one pair of wheels. Setting [num_wheel_pairs] to 1");
    }

    // Dynamic properties
    impl_->max_wheel_accel_ = _sdf->Get<double>("wheelAccel", 0.0).first;
    impl_->max_wheel_torque_ = _sdf->Get<double>("WheelTorque", 5.0).first;

    // Dynamic PID properties
    impl_->pid_lr.p_gain = _sdf->Get<double>("Wheel_LR_P", 1.0).first;
    impl_->pid_lr.i_gain = _sdf->Get<double>("Wheel_LR_I", 0.0).first;
    impl_->pid_lr.d_gain = _sdf->Get<double>("Wheel_LR_D", 0.0).first;

    impl_->pid_lf.p_gain = _sdf->Get<double>("Wheel_LF_P", 1.0).first;
    impl_->pid_lf.i_gain = _sdf->Get<double>("Wheel_LF_I", 0.0).first;
    impl_->pid_lf.d_gain = _sdf->Get<double>("Wheel_LF_D", 0.0).first;

    impl_->pid_rf.p_gain = _sdf->Get<double>("Wheel_RF_P", 1.0).first;
    impl_->pid_rf.i_gain = _sdf->Get<double>("Wheel_RF_I", 0.0).first;
    impl_->pid_rf.d_gain = _sdf->Get<double>("Wheel_RF_D", 0.0).first;

    impl_->pid_rr.p_gain = _sdf->Get<double>("Wheel_RR_P", 1.0).first;
    impl_->pid_rr.i_gain = _sdf->Get<double>("Wheel_RR_I", 0.0).first;
    impl_->pid_rr.d_gain = _sdf->Get<double>("Wheel_RR_D", 0.0).first;

    // Get joints and Kinematic properties
    std::vector<gazebo::physics::JointPtr> left_rear_joints, left_front_joints,  right_front_joints, right_rear_joints;

    for (auto left_rear_joint_elem = _sdf->GetElement("LeftRear"); left_rear_joint_elem != nullptr; left_rear_joint_elem = left_rear_joint_elem->GetNextElement("LeftRear"))
    {
      auto left_rear_joint_name = left_rear_joint_elem->Get<std::string>();
      auto left_rear_joint = _model->GetJoint(left_rear_joint_name);
      if (!left_rear_joint)
      {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint [%s] not found, plugin will not work.", left_rear_joint_name.c_str());
        impl_->ros_node_.reset();
        return;
      }
      
      left_rear_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
      left_rear_joints.push_back(left_rear_joint);
    }

    for (auto left_front_joint_elem = _sdf->GetElement("LeftFront"); left_front_joint_elem != nullptr; left_front_joint_elem = left_front_joint_elem->GetNextElement("LeftFront"))
    {
      auto left_front_joint_name = left_front_joint_elem->Get<std::string>();
      auto left_front_joint = _model->GetJoint(left_front_joint_name);
      if (!left_front_joint)
      {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint [%s] not found, plugin will not work.", left_front_joint_name.c_str());
        impl_->ros_node_.reset();
        return;
      }
      
      left_front_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
      left_front_joints.push_back(left_front_joint);
    }
    
    for (auto right_front_joint_elem = _sdf->GetElement("RightFront"); right_front_joint_elem != nullptr; right_front_joint_elem = right_front_joint_elem->GetNextElement("RightFront"))
    {
      auto right_front_joint_name = right_front_joint_elem->Get<std::string>();
      auto right_front_joint = _model->GetJoint(right_front_joint_name);
      if (!right_front_joint)
      {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint [%s] not found, plugin will not work.", right_front_joint_name.c_str());
        impl_->ros_node_.reset();
        return;
      }
      
      right_front_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
      right_front_joints.push_back(right_front_joint);
    }

    for (auto right_rear_joint_elem = _sdf->GetElement("RightRear"); right_rear_joint_elem != nullptr; right_rear_joint_elem = right_rear_joint_elem->GetNextElement("RightRear"))
    {
      auto right_rear_joint_name = right_rear_joint_elem->Get<std::string>();
      auto right_rear_joint = _model->GetJoint(right_rear_joint_name);
      if (!right_rear_joint)
      {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint [%s] not found, plugin will not work.", right_rear_joint_name.c_str());
        impl_->ros_node_.reset();
        return;
      }
      
      right_rear_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
      right_rear_joints.push_back(right_rear_joint);
    }

    unsigned int index;
    for (index = 0; index < impl_->num_wheel_pairs_; ++index)
    {
      impl_->joints_.push_back(left_rear_joints[index]);
      impl_->joints_.push_back(left_front_joints[index]);
      impl_->joints_.push_back(right_front_joints[index]);
      impl_->joints_.push_back(right_rear_joints[index]);
    }
    
    index = 0;
    impl_->wheel_w_separation_.assign(impl_->num_wheel_pairs_, 0.34);
    for (auto wheel_w_separation = _sdf->GetElement("WheelSeparationW"); wheel_w_separation != nullptr; wheel_w_separation = wheel_w_separation->GetNextElement("WheelSeparationW"))
    {
      if (index >= impl_->num_wheel_pairs_)
      {
        RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <WheelSeparationW>");
        break;
      }
      
      impl_->wheel_w_separation_[index] = wheel_w_separation->Get<double>();
      RCLCPP_INFO(impl_->ros_node_->get_logger(), "Wheel pair %i separation width set to [%fm]", index + 1, impl_->wheel_w_separation_[index]);
      index++;
    }

    index = 0;
    impl_->wheel_l_separation_.assign(impl_->num_wheel_pairs_, 0.34);
    for (auto wheel_l_separation = _sdf->GetElement("WheelSeparationL"); wheel_l_separation != nullptr; wheel_l_separation = wheel_l_separation->GetNextElement("WheelSeparationL"))
    {
      if (index >= impl_->num_wheel_pairs_)
      {
        RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <WheelSeparationL>");
        break;
      }
      
      impl_->wheel_l_separation_[index] = wheel_l_separation->Get<double>();
      RCLCPP_INFO(impl_->ros_node_->get_logger(), "Wheel pair %i separation legth set to [%fm]", index + 1, impl_->wheel_l_separation_[index]);
      index++;
    }
    
    index = 0;
    impl_->wheel_diameter_.assign(impl_->num_wheel_pairs_, 0.15);
    for (auto wheel_diameter = _sdf->GetElement("wheelDiameter"); wheel_diameter != nullptr; wheel_diameter = wheel_diameter->GetNextElement("wheelDiameter"))
    {
      if (index >= impl_->num_wheel_pairs_)
      {
        RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheelDiameter>");
        break;
      }
      
      impl_->wheel_diameter_[index] = wheel_diameter->Get<double>();
      RCLCPP_INFO(impl_->ros_node_->get_logger(), "Wheel pair %i diameter set to [%fm]", index + 1, impl_->wheel_diameter_[index]);
      index++;
    }
    
    impl_->wheel_speed_instr_.assign(4 * impl_->num_wheel_pairs_, 0);
    impl_->desired_wheel_speed_.assign(4 * impl_->num_wheel_pairs_, 0);
    
    // Update rate
    auto update_rate = _sdf->Get<double>("odometryRate", 20.0).first;
    if (update_rate > 0.0)
    {
      impl_->update_period_ = 1.0 / update_rate;
    }
    else
    {
      impl_->update_period_ = 0.0;
    }

    impl_->cmd_topic_ = _sdf->Get<std::string>("commandTopic", "cmd_vel").first;
    impl_->last_update_time_ = _model->GetWorld()->SimTime();
    impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(impl_->cmd_topic_, qos.get_subscription_qos(impl_->cmd_topic_, rclcpp::QoS(1)),
                                                                                           std::bind(&GazeboMecanumRosDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));
    
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->cmd_vel_sub_->get_topic_name());
    // Odometry
    impl_->odometry_frame_ = _sdf->Get<std::string>("odometryFrame", "odom").first;
    impl_->robot_base_frame_ = _sdf->Get<std::string>("robotBaseFrame", "base_footprint").first;
    
    // Advertise odometry topic
    impl_->publish_odom_ = _sdf->Get<bool>("publishOdom", false).first;
    if(impl_->publish_odom_)
    {
      impl_->odometry_topic_ = _sdf->Get<std::string>("odometryTopic", "odom").first;
      impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(impl_->odometry_topic_, qos.get_publisher_qos(impl_->odometry_topic_, rclcpp::QoS(1)));
      RCLCPP_INFO(impl_->ros_node_->get_logger(), "Advertise odometry on [%s]", impl_->odometry_pub_->get_topic_name());
    }
    
    // Create TF broadcaster if needed
    impl_->publish_wheel_tf_ = _sdf->Get<bool>("publishWheelTF", false).first;
    impl_->publish_odom_tf_ = _sdf->Get<bool>("publishOdomTF", false).first;
    if(impl_->publish_wheel_tf_ || impl_->publish_odom_tf_)
    {
      impl_->transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);
      if(impl_->publish_odom_tf_)
      {
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
                                                                                                        impl_->robot_base_frame_.c_str());
      }
      
      for (index = 0; index < impl_->num_wheel_pairs_; ++index)
      {
        if(impl_->publish_wheel_tf_)
        {
          RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing wheel transforms between [%s], [%s] , [%s] , [%s], [%s]",
                                                       impl_->robot_base_frame_.c_str(),
                                                       impl_->joints_[2 * index + GazeboMecanumRosDrivePrivate::LEFT_REAR]->GetName().c_str(),
                                                       impl_->joints_[2 * index + GazeboMecanumRosDrivePrivate::LEFT_FRONT]->GetName().c_str(),
                                                       impl_->joints_[2 * index + GazeboMecanumRosDrivePrivate::RIGHT_FRONT]->GetName().c_str(),
                                                       impl_->joints_[2 * index + GazeboMecanumRosDrivePrivate::RIGHT_REAR]->GetName().c_str());
        }
      }
    }

    impl_->isRollerModel_ = _sdf->Get<bool>("isRollerModel", true).first;
    impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
    impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
    impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;
    
    // Listen to the update event (broadcast every simulation iteration)
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboMecanumRosDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
  }
  
  void GazeboMecanumRosDrive::Reset()
  {
    impl_->last_update_time_ = impl_->joints_[GazeboMecanumRosDrivePrivate::LEFT_REAR]->GetWorld()->SimTime();
    for(unsigned int i = 0; i < impl_->num_wheel_pairs_; ++i)
    {
      if (impl_->joints_[2 * i + GazeboMecanumRosDrivePrivate::LEFT_REAR] && impl_->joints_[2 * i + GazeboMecanumRosDrivePrivate::LEFT_FRONT] &&
          impl_->joints_[2 * i + GazeboMecanumRosDrivePrivate::RIGHT_FRONT] && impl_->joints_[2 * i + GazeboMecanumRosDrivePrivate::RIGHT_REAR])
      {
        impl_->joints_[2 * i + GazeboMecanumRosDrivePrivate::LEFT_REAR]->SetParam("fmax", 0, impl_->max_wheel_torque_);
        impl_->joints_[2 * i + GazeboMecanumRosDrivePrivate::LEFT_FRONT]->SetParam("fmax", 0, impl_->max_wheel_torque_);
        impl_->joints_[2 * i + GazeboMecanumRosDrivePrivate::RIGHT_FRONT]->SetParam("fmax", 0, impl_->max_wheel_torque_);
        impl_->joints_[2 * i + GazeboMecanumRosDrivePrivate::RIGHT_REAR]->SetParam("fmax", 0, impl_->max_wheel_torque_);
      }
    }
    
    impl_->pose_encoder_.x = 0;
    impl_->pose_encoder_.y = 0;
    impl_->pose_encoder_.theta = 0;
    impl_->target_x_ = 0;
    impl_->target_rot_ = 0;
  }

  void GazeboMecanumRosDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
  {
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("GazeboRosDiffDrivePrivate::OnUpdate");
  #endif
    
    double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
    if(seconds_since_last_update < update_period_)
    {
      return;
    }

  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("PublishOdometryMsg");
  #endif
    if(publish_odom_)
    {
      PublishOdometryMsg(_info.simTime);
    }
    #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("PublishWheelsTf");
  #endif
    if(publish_wheel_tf_)
    {
      PublishWheelsTf(_info.simTime);
    }
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("PublishOdometryTf");
  #endif
    if(publish_odom_tf_)
    {
      PublishOdometryTf(_info.simTime);
    }
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("UpdateWheelVelocities");
  #endif
  // Update robot in case new velocities have been requested
  UpdateWheelVelocities();
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
  #endif
    // Current speed
    std::vector<double> current_speed(4 * num_wheel_pairs_);
    for(unsigned int i = 0; i < num_wheel_pairs_; ++i)
    {
      current_speed[2 * i + LEFT_REAR] = joints_[2 * i + LEFT_REAR]->GetVelocity(0);
      current_speed[2 * i + LEFT_FRONT] = joints_[2 * i + LEFT_FRONT]->GetVelocity(0);
      current_speed[2 * i + RIGHT_FRONT] = joints_[2 * i + RIGHT_FRONT]->GetVelocity(0);
      current_speed[2 * i + RIGHT_REAR] = joints_[2 * i + RIGHT_REAR]->GetVelocity(0);
    }

    // If max_accel == 0, or target speed is reached
    for(unsigned int i = 0; i < num_wheel_pairs_; ++i)
    {
      if(max_wheel_accel_ == 0 ||
        ((fabs(desired_wheel_speed_[2 * i + LEFT_REAR] - current_speed[2 * i + LEFT_REAR]) < 0.01) &&
         (fabs(desired_wheel_speed_[2 * i + LEFT_FRONT] - current_speed[2 * i + LEFT_FRONT]) < 0.01) &&
         (fabs(desired_wheel_speed_[2 * i + RIGHT_FRONT] - current_speed[2 * i + RIGHT_FRONT]) < 0.01) &&
         (fabs(desired_wheel_speed_[2 * i + RIGHT_REAR] - current_speed[2 * i + RIGHT_REAR]) < 0.01)))
      {
        gazebo_PID_WHEEL_(joints_[2 * i + LEFT_REAR], pid_lr, desired_wheel_speed_[2 * i + LEFT_REAR]);
        gazebo_PID_WHEEL_(joints_[2 * i + LEFT_FRONT], pid_lf, desired_wheel_speed_[2 * i + LEFT_FRONT]);
        gazebo_PID_WHEEL_(joints_[2 * i + RIGHT_FRONT], pid_rf, desired_wheel_speed_[2 * i + RIGHT_FRONT]);
        gazebo_PID_WHEEL_(joints_[2 * i + RIGHT_REAR], pid_rr, desired_wheel_speed_[2 * i + RIGHT_REAR]);
      }
      else
      {
        if(desired_wheel_speed_[2 * i + LEFT_REAR] >= current_speed[2 * i + LEFT_REAR])
        {
          wheel_speed_instr_[2 * i + LEFT_REAR] += fmin(desired_wheel_speed_[2 * i + LEFT_REAR] - current_speed[2 * i + LEFT_REAR], max_wheel_accel_ * seconds_since_last_update);
        }
        else
        {
          wheel_speed_instr_[2 * i + LEFT_REAR] += fmax(desired_wheel_speed_[2 * i + LEFT_REAR] - current_speed[2 * i + LEFT_REAR], -max_wheel_accel_ * seconds_since_last_update);
        }

        if(desired_wheel_speed_[2 * i + LEFT_FRONT] >= current_speed[2 * i + LEFT_FRONT])
        {
          wheel_speed_instr_[2 * i + LEFT_FRONT] += fmin(desired_wheel_speed_[2 * i + LEFT_FRONT] - current_speed[2 * i + LEFT_FRONT], max_wheel_accel_ * seconds_since_last_update);
        }
        else
        {
          wheel_speed_instr_[2 * i + LEFT_FRONT] += fmax(desired_wheel_speed_[2 * i + LEFT_FRONT] - current_speed[2 * i + LEFT_FRONT], -max_wheel_accel_ * seconds_since_last_update);
        }

        if (desired_wheel_speed_[2 * i + RIGHT_FRONT] > current_speed[2 * i + RIGHT_FRONT])
        {
          wheel_speed_instr_[2 * i + RIGHT_FRONT] += fmin(desired_wheel_speed_[2 * i + RIGHT_FRONT] - current_speed[2 * i + RIGHT_FRONT], max_wheel_accel_ * seconds_since_last_update);
        }
        else
        {
          wheel_speed_instr_[2 * i + RIGHT_FRONT] += fmax(desired_wheel_speed_[2 * i + RIGHT_FRONT] - current_speed[2 * i + RIGHT_FRONT], -max_wheel_accel_ * seconds_since_last_update);
        }

        if (desired_wheel_speed_[2 * i + RIGHT_REAR] > current_speed[2 * i + RIGHT_REAR])
        {
          wheel_speed_instr_[2 * i + RIGHT_REAR] += fmin(desired_wheel_speed_[2 * i + RIGHT_REAR] - current_speed[2 * i + RIGHT_REAR], max_wheel_accel_ * seconds_since_last_update);
        }
        else
        {
          wheel_speed_instr_[2 * i + RIGHT_REAR] += fmax(desired_wheel_speed_[2 * i + RIGHT_REAR] - current_speed[2 * i + RIGHT_REAR], -max_wheel_accel_ * seconds_since_last_update);
        }

        gazebo_PID_WHEEL_(joints_[2 * i + LEFT_REAR], pid_lr, desired_wheel_speed_[2 * i + LEFT_REAR]);
        gazebo_PID_WHEEL_(joints_[2 * i + LEFT_FRONT], pid_lf, desired_wheel_speed_[2 * i + LEFT_FRONT]);
        gazebo_PID_WHEEL_(joints_[2 * i + RIGHT_FRONT], pid_rf, desired_wheel_speed_[2 * i + RIGHT_FRONT]);
        gazebo_PID_WHEEL_(joints_[2 * i + RIGHT_REAR], pid_rr, desired_wheel_speed_[2 * i + RIGHT_REAR]);
      }
    }

    if(!isRollerModel_)
    {
      ignition::math::Pose3d pose = model_->WorldPose();
      float yaw = pose.Rot().Yaw();
      calkinematics(cal_LineVel_);
      model_->SetLinearVel(ignition::math::Vector3d(cal_LineVel_.vel_x * cosf(yaw) - cal_LineVel_.vel_y * sinf(yaw), 
                                                     cal_LineVel_.vel_y * cosf(yaw) + cal_LineVel_.vel_x * sinf(yaw), 0));
      model_->SetAngularVel(ignition::math::Vector3d(0, 0, cal_LineVel_.vel_th));
    }

    last_update_time_ = _info.simTime;
  }

  void GazeboMecanumRosDrivePrivate::UpdateWheelVelocities()
  {
    std::lock_guard<std::mutex> scoped_lock(lock_);

    double vx = target_x_;
    double vy = target_y_;
    double va = target_rot_;

    for (unsigned int i = 0; i < num_wheel_pairs_; ++i)
    {
      desired_wheel_speed_[2 * i + LEFT_REAR] = (vx + vy - va * (wheel_w_separation_[i] + wheel_l_separation_[i]) / 2.0) / wheel_diameter_[i] * 2;
      desired_wheel_speed_[2 * i + LEFT_FRONT] = (vx - vy - va * (wheel_w_separation_[i] + wheel_l_separation_[i]) / 2.0) / wheel_diameter_[i] * 2;
      desired_wheel_speed_[2 * i + RIGHT_FRONT] = (vx + vy + va * (wheel_w_separation_[i] + wheel_l_separation_[i]) / 2.0) / wheel_diameter_[i] * 2;
      desired_wheel_speed_[2 * i + RIGHT_REAR] = (vx - vy + va * (wheel_w_separation_[i] + wheel_l_separation_[i]) / 2.0) / wheel_diameter_[i] * 2;
    }
  }

  void GazeboMecanumRosDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
  {
    std::lock_guard<std::mutex> scoped_lock(lock_);
    target_x_ = _msg->linear.x;
    target_y_ = _msg->linear.y;
    target_rot_ = _msg->angular.z;
  }

  void GazeboMecanumRosDrivePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = odometry_frame_;
    msg.child_frame_id = robot_base_frame_;
    msg.transform.translation =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
    msg.transform.rotation = odom_.pose.pose.orientation;

    transform_broadcaster_->sendTransform(msg);
  }

  void GazeboMecanumRosDrivePrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
  {
    for (unsigned int i = 0; i < 4 * num_wheel_pairs_; ++i)
    {
      auto pose_wheel = joints_[i]->GetChild()->RelativePose();

      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
      msg.header.frame_id = joints_[i]->GetParent()->GetName();
      msg.child_frame_id = joints_[i]->GetChild()->GetName();
      msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_wheel.Pos());
      msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose_wheel.Rot());

      transform_broadcaster_->sendTransform(msg);
    }
  }

  void GazeboMecanumRosDrivePrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
  {
    auto pose = model_->WorldPose();
    odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
    odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

    // Get velocity in odom frame
    auto linear = model_->WorldLinearVel();
    odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

    // Convert velocity to child_frame_id(aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
    // Set covariance
    odom_.pose.covariance[0] = covariance_[0];
    odom_.pose.covariance[7] = covariance_[1];
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = covariance_[2];

    odom_.twist.covariance[0] = covariance_[0];
    odom_.twist.covariance[7] = covariance_[1];
    odom_.twist.covariance[14] = 1000000000000.0;
    odom_.twist.covariance[21] = 1000000000000.0;
    odom_.twist.covariance[28] = 1000000000000.0;
    odom_.twist.covariance[35] = covariance_[2];

    // Set header
    odom_.header.frame_id = odometry_frame_;
    odom_.child_frame_id = robot_base_frame_;
    odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

    // Publish
    odometry_pub_->publish(odom_);
  }

  void GazeboMecanumRosDrivePrivate::calkinematics(linear_vel &line_vel)
  {
    double wheel_encoder[4];
    wheel_encoder[LEFT_REAR] = joints_[LEFT_REAR]->GetVelocity(0);
    wheel_encoder[LEFT_FRONT] = joints_[LEFT_FRONT]->GetVelocity(0);
    wheel_encoder[RIGHT_FRONT] = joints_[RIGHT_FRONT]->GetVelocity(0);
    wheel_encoder[RIGHT_REAR] = joints_[RIGHT_REAR]->GetVelocity(0);

    double l = 1.0 / (2 * (wheel_w_separation_[0] + wheel_l_separation_[0]));

    line_vel.vel_x = (wheel_encoder[LEFT_REAR] + wheel_encoder[LEFT_FRONT] + wheel_encoder[RIGHT_FRONT] + wheel_encoder[RIGHT_REAR]) * wheel_diameter_[0] / 2;
    line_vel.vel_y = (wheel_encoder[LEFT_REAR] - wheel_encoder[LEFT_FRONT] + wheel_encoder[RIGHT_FRONT] - wheel_encoder[RIGHT_REAR]) * wheel_diameter_[0] / 2;
    line_vel.vel_th = (-wheel_encoder[LEFT_REAR] - wheel_encoder[LEFT_FRONT] + wheel_encoder[RIGHT_FRONT] + wheel_encoder[RIGHT_REAR]) * l * wheel_diameter_[0] / 2;
  }

  void GazeboMecanumRosDrivePrivate::gazebo_PID_WHEEL_(gazebo::physics::JointPtr joint, pid_parameter &pid, double vel)
  {
    gazebo::common::PID wheel_pid = gazebo::common::PID(pid.p_gain, pid.i_gain, pid.d_gain);
    model_->GetJointController()->SetVelocityPID(joint->GetScopedName(), wheel_pid);
    model_->GetJointController()->SetVelocityTarget(joint->GetScopedName(), vel);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboMecanumRosDrive)
}  // namespace gazebo_plugins