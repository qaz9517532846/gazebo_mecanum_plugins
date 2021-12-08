#include <gazebo_mecanum_plugins/gazebo_ros_mecanum_drive.h>

namespace gazebo
{
    enum
    {
        LEFT_REAR = 0,
        LEFT_FRONT = 1,
        RIGHT_FRONT = 2,
        RIGHT_REAR = 3,
    };

    GazeboRosMecanumDrive::GazeboRosMecanumDrive()
    {

    }
    
    GazeboRosMecanumDrive::~GazeboRosMecanumDrive()
    {
        FiniChild();
    }
    
    // Load the controller
    void GazeboRosMecanumDrive::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
        parent_ = parent;
        /* Parse parameters */
        
        robot_namespace_ = "";
        if(!sdf->HasElement("robotNamespace"))
        {
            ROS_INFO_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin missing <robotNamespace>, "
                           "defaults to \"%s\"", robot_namespace_.c_str());
        }
        else
        {
            robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
        }
        
        command_topic_ = "cmd_vel";
        if(!sdf->HasElement("commandTopic"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <commandTopic>, "
                           "defaults to \"%s\"",
                            robot_namespace_.c_str(), command_topic_.c_str());
        }
        else
        {
            command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
        }

        odometry_topic_ = "odom";
        if(!sdf->HasElement("odometryTopic"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <odometryTopic>, "
                           "defaults to \"%s\"",
                            robot_namespace_.c_str(), odometry_topic_.c_str());
        }
        else
        {
            odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
        }
        
        odometry_frame_ = "odom";
        if(!sdf->HasElement("odometryFrame"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <odometryFrame>, "
                           "defaults to \"%s\"",
                           robot_namespace_.c_str(), odometry_frame_.c_str());
        }
        else
        {
            odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
        }

        robot_base_frame_ = "base_footprint";
        if(!sdf->HasElement("robotBaseFrame"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <robotBaseFrame>, "
                           "defaults to \"%s\"",
                           robot_namespace_.c_str(), robot_base_frame_.c_str());
        }
        else
        {
            robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
        }

        left_rear_ = "left_rear_joint";
        if(!sdf->HasElement("LeftRear"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <LeftRear>, "
                           "defaults to \"%s\"",
                           robot_namespace_.c_str(), left_rear_.c_str());
        }
        else
        {
            left_rear_ = sdf->GetElement("LeftRear")->Get<std::string>();
        }

        left_front_ = "left_front_joint";
        if(!sdf->HasElement("LeftFront"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <LeftFront>, "
                           "defaults to \"%s\"",
                           robot_namespace_.c_str(), left_front_.c_str());
        }
        else
        {
            left_front_ = sdf->GetElement("LeftFront")->Get<std::string>();
        }

        right_front_ = "right_front_joint";
        if(!sdf->HasElement("RightFront"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <RightFront>, "
                           "defaults to \"%s\"",
                           robot_namespace_.c_str(), right_front_.c_str());
        }
        else
        {
            right_front_ = sdf->GetElement("RightFront")->Get<std::string>();
        }

        right_rear_ = "right_rear_joint";
        if(!sdf->HasElement("RightRear"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <RightRear>, "
                           "defaults to \"%s\"",
                           robot_namespace_.c_str(), right_rear_.c_str());
        }
        else
        {
            right_rear_ = sdf->GetElement("RightRear")->Get<std::string>();
        }
        
        odometry_rate_ = 20.0;
        if(!sdf->HasElement("odometryRate"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <odometryRate>, "
                           "defaults to %f",
                            robot_namespace_.c_str(), odometry_rate_);
        }
        else
        {
            odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
        }

        publishWheelTF_ = true;
        if(!sdf->HasElement("publishWheelTF"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <publishWheelTF>, "
                           "defaults to %d",
                            robot_namespace_.c_str(), publishWheelTF_);
        }
        else
        {
            publishWheelTF_ = sdf->GetElement("publishWheelTF")->Get<bool>();
        }

        publishWheelJointState_ = true;
        if(!sdf->HasElement("publishWheelJointState"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <publishWheelJointState>, "
                           "defaults to %d",
                            robot_namespace_.c_str(), publishWheelJointState_);
        }
        else
        {
            publishWheelJointState_ = sdf->GetElement("publishWheelJointState")->Get<bool>();
        }

        wheel_torque = 50.0;
        if(!sdf->HasElement("WheelTorque"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <WheelTorque>, "
                           "defaults to %f",
                            robot_namespace_.c_str(), wheel_torque);
        }
        else
        {
            wheel_torque = sdf->GetElement("WheelTorque")->Get<double>();
        }

        wheel_separation_w = 0.5;
        if(!sdf->HasElement("WheelSeparationW"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <WheelSeparationW>, "
                           "defaults to %f",
                            robot_namespace_.c_str(), wheel_separation_w);
        }
        else
        {
            wheel_separation_w = sdf->GetElement("WheelSeparationW")->Get<double>();
        }

        wheel_separation_l = 0.5;
        if(!sdf->HasElement("WheelSeparationL"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <WheelSeparationL>, "
                           "defaults to %f",
                            robot_namespace_.c_str(), wheel_separation_l);
        }
        else
        {
            wheel_separation_w = sdf->GetElement("WheelSeparationL")->Get<double>();
        }
        
        wheel_accel_ = 0;
        if(!sdf->HasElement("wheelAccel"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <wheelAccel>, "
                           "defaults to %f",
                            robot_namespace_.c_str(), wheel_accel_);
        }
        else
        {
            wheel_accel_ = sdf->GetElement("wheelAccel")->Get<double>();
        }

        wheel_diameter_ = 0.13;
        if(!sdf->HasElement("wheelDiameter"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <wheelDiameter>, "
                           "defaults to %f",
                            robot_namespace_.c_str(), wheel_diameter_);
        }
        else
        {
            wheel_diameter_ = sdf->GetElement("wheelDiameter")->Get<double>();
        }

        publishOdomentry_ = true;
        if(!sdf->HasElement("publishOdom"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <publishOdom>, "
                           "defaults to %d",
                            robot_namespace_.c_str(), publishOdomentry_);
        }
        else
        {
            publishOdomentry_ = sdf->GetElement("publishOdom")->Get<bool>();
        }

        isRollerModel_ = false;
        if(!sdf->HasElement("isRollerModel"))
        {
            ROS_WARN_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = %s) missing <isRollerModel>, "
                           "defaults to %d",
                            robot_namespace_.c_str(), isRollerModel_);
        }
        else
        {
            isRollerModel_ = sdf->GetElement("isRollerModel")->Get<bool>();
        }

        joints_.resize(4);
        joints_[LEFT_REAR] = parent_->GetJoint(left_rear_);
        joints_[LEFT_FRONT] = parent_->GetJoint(left_front_);
        joints_[RIGHT_FRONT] = parent_->GetJoint(right_front_);
        joints_[RIGHT_REAR] = parent_->GetJoint(right_rear_);

        joints_[LEFT_REAR]->SetParam("fmax", 0, wheel_torque);
        joints_[LEFT_FRONT]->SetParam("fmax", 0, wheel_torque);
        joints_[RIGHT_FRONT]->SetParam("fmax", 0, wheel_torque);
        joints_[RIGHT_REAR]->SetParam("fmax", 0, wheel_torque);

    #if GAZEBO_MAJOR_VERSION >= 8
        last_odom_publish_time_ = parent_->GetWorld()->SimTime();
    #else
        last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
    #endif

    #if GAZEBO_MAJOR_VERSION >= 8
        last_odom_pose_ = parent_->WorldPose();
    #else
        last_odom_pose_ = parent_->GetWorldPose().Ign();
    #endif

        wheel_speed_[LEFT_REAR] = 0;
        wheel_speed_[LEFT_FRONT] = 0;
        wheel_speed_[RIGHT_FRONT] = 0;
        wheel_speed_[RIGHT_REAR] = 0;

        wheel_speed_instr_[LEFT_REAR] = 0;
        wheel_speed_instr_[LEFT_FRONT] = 0;
        wheel_speed_instr_[RIGHT_FRONT] = 0;
        wheel_speed_instr_[RIGHT_REAR] = 0;

        x_ = 0;
        y_ = 0;
        rot_ = 0;
        alive_ = true;

        // Ensure that ROS has been initialized and subscribe to cmd_vel
        if(!ros::isInitialized())
        {
            ROS_FATAL_STREAM_NAMED("MecanumDiffDrive", "MecanumDiffDrivePlugin (ns = " << robot_namespace_
                                    << "). A ROS node for Gazebo has not been initialized, "
                                    << "unable to load plugin. Load the Gazebo system plugin "
                                    << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }
        
        rosnode_.reset(new ros::NodeHandle(robot_namespace_));
        ROS_DEBUG_NAMED("MecanumDiffDrive", "OCPlugin (%s) has started", robot_namespace_.c_str());
        tf_prefix_ = tf::getPrefixParam(*rosnode_);
        transform_broadcaster_.reset(new tf::TransformBroadcaster());
        
        // subscribe to the odometry topic
        ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                                                                                       boost::bind(&GazeboRosMecanumDrive::cmdVelCallback, this, _1),
                                                                                       ros::VoidPtr(), &queue_);

        vel_sub_ = rosnode_->subscribe(so);
        odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

        if (this->publishWheelJointState_)
        {
            joint_state_publisher_ = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 1000);
            ROS_INFO_NAMED("MecanumDiffDrive", "%s: Advertise joint_states", robot_namespace_.c_str());
        }

        // start custom queue for diff drive
        callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosMecanumDrive::QueueThread, this));

        // listen to the update event (broadcast every simulation iteration)
        update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosMecanumDrive::UpdateChild, this));
    }

    void GazeboRosMecanumDrive::publishWheelJointState()
    {
        ros::Time current_time = ros::Time::now();
        
        joint_state_.header.stamp = current_time;
        joint_state_.name.resize(joints_.size());
        joint_state_.position.resize(joints_.size());
        
        for(int i = 0; i < 4; i++)
        {
            physics::JointPtr joint = joints_[i];

    #if GAZEBO_MAJOR_VERSION >= 8
            double position = joint->Position(0);
    #else
            double position = joint->GetAngle(0).Radian();
    #endif
            joint_state_.name[i] = joint->GetName();
            joint_state_.position[i] = position;
        }
        joint_state_publisher_.publish(joint_state_);
    }
    
    void GazeboRosMecanumDrive::publishWheelTF()
    {
        ros::Time current_time = ros::Time::now();
        for(int i = 0; i < 4; i++)
        {
            std::string wheel_frame = joints_[i]->GetChild()->GetName();
            std::string wheel_parent_frame = joints_[i]->GetParent()->GetName();
    #if GAZEBO_MAJOR_VERSION >= 8
            ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->RelativePose();
    #else
            ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->GetRelativePose().Ign();
    #endif
            tf::Quaternion qt(poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W());
            tf::Vector3 vt(poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z());
            tf::Transform tfWheel(qt, vt);
            transform_broadcaster_->sendTransform(tf::StampedTransform(tfWheel, current_time, wheel_parent_frame, wheel_frame));
        }
    }

    // Update the controller
    void GazeboRosMecanumDrive::UpdateChild()
    {
        boost::mutex::scoped_lock scoped_lock(lock);
        for(int i = 0; i < 4; i++)
        {
            if(fabs(wheel_torque - joints_[i]->GetParam("fmax", 0)) > 1e-6)
            {
                joints_[i]->SetParam ("fmax", 0, wheel_torque);
            }
        }

    #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = parent_->WorldPose();
    #else
        ignition::math::Pose3d pose = parent_->GetWorldPose().Ign();
    #endif

        float yaw = pose.Rot().Yaw();
    
        if(odometry_rate_ > 0.0)
        {
    #if GAZEBO_MAJOR_VERSION >= 8
            common::Time current_time = parent_->GetWorld()->SimTime();
    #else
            common::Time current_time = parent_->GetWorld()->GetSimTime();
    #endif
            double seconds_since_last_update = (current_time - last_odom_publish_time_).Double();
            
            if(seconds_since_last_update > (1.0 / odometry_rate_))
            {
                if(publishWheelTF_) publishWheelTF();
                if(publishWheelJointState_) publishWheelJointState();

                getWheelVelocity();

                double current_speed[4];
                current_speed[LEFT_REAR] = joints_[LEFT_REAR]->GetVelocity(0);
                current_speed[LEFT_FRONT] = joints_[LEFT_FRONT]->GetVelocity(0);
                current_speed[RIGHT_FRONT] = joints_[RIGHT_FRONT]->GetVelocity(0);
                current_speed[RIGHT_REAR] = joints_[RIGHT_REAR]->GetVelocity(0);

                if(wheel_accel_ == 0 ||
                  (fabs(wheel_speed_[LEFT_REAR] - current_speed[LEFT_REAR] ) < 0.01) ||
                  (fabs(wheel_speed_[LEFT_FRONT] - current_speed[LEFT_FRONT] ) < 0.01) ||
                  (fabs(wheel_speed_[RIGHT_FRONT] - current_speed[RIGHT_FRONT] ) < 0.01) ||
                  (fabs(wheel_speed_[RIGHT_REAR] - current_speed[RIGHT_REAR] ) < 0.01))
                  {
                      //if max_accel == 0, or target speed is reached
                      joints_[LEFT_REAR]->SetParam("vel", 0, wheel_speed_[LEFT_REAR]);
                      joints_[LEFT_FRONT]->SetParam("vel", 0, wheel_speed_[LEFT_FRONT]);
                      joints_[RIGHT_FRONT]->SetParam("vel", 0, wheel_speed_[RIGHT_FRONT]);
                      joints_[RIGHT_REAR]->SetParam("vel", 0, wheel_speed_[RIGHT_REAR]);
                  }
                  else
                  {
                    if(wheel_speed_[LEFT_REAR] > current_speed[LEFT_REAR])
                        wheel_speed_instr_[LEFT_REAR] += fmin(wheel_speed_[LEFT_REAR] - current_speed[LEFT_REAR],  wheel_accel_ * seconds_since_last_update);
                    else
                        wheel_speed_instr_[LEFT_REAR] += fmax(wheel_speed_[LEFT_REAR] - current_speed[LEFT_REAR], -wheel_accel_ * seconds_since_last_update);

                    if(wheel_speed_[LEFT_FRONT] > current_speed[LEFT_FRONT])
                        wheel_speed_instr_[LEFT_FRONT] += fmin(wheel_speed_[LEFT_FRONT] - current_speed[LEFT_FRONT], wheel_accel_ * seconds_since_last_update);
                    else
                        wheel_speed_instr_[LEFT_FRONT] += fmax(wheel_speed_[LEFT_FRONT] - current_speed[LEFT_FRONT], -wheel_accel_ * seconds_since_last_update);

                    if(wheel_speed_[RIGHT_FRONT] > current_speed[RIGHT_FRONT])
                        wheel_speed_instr_[RIGHT_FRONT] += fmin(wheel_speed_[RIGHT_FRONT] - current_speed[RIGHT_FRONT], wheel_accel_ * seconds_since_last_update);
                    else
                        wheel_speed_instr_[RIGHT_FRONT] += fmax(wheel_speed_[RIGHT_FRONT] - current_speed[RIGHT_FRONT], -wheel_accel_ * seconds_since_last_update);

                    if(wheel_speed_[RIGHT_REAR] > current_speed[RIGHT_REAR])
                        wheel_speed_instr_[RIGHT_REAR] += fmin(wheel_speed_[RIGHT_REAR] - current_speed[RIGHT_REAR], wheel_accel_ * seconds_since_last_update);
                    else
                        wheel_speed_instr_[RIGHT_REAR] += fmax(wheel_speed_[RIGHT_REAR] - current_speed[RIGHT_REAR], -wheel_accel_ * seconds_since_last_update);

                    joints_[LEFT_REAR]->SetParam("vel", 0, wheel_speed_instr_[LEFT_REAR]);
                    joints_[LEFT_FRONT]->SetParam("vel", 0, wheel_speed_instr_[LEFT_FRONT]);
                    joints_[RIGHT_FRONT]->SetParam("vel", 0, wheel_speed_instr_[RIGHT_FRONT]);
                    joints_[RIGHT_REAR]->SetParam("vel", 0, wheel_speed_instr_[RIGHT_REAR]);
                }

                if(!isRollerModel_)
                {
                    calkinematics(cal_LineVel_);
                    parent_->SetLinearVel(ignition::math::Vector3d(cal_LineVel_.vel_x * cosf(yaw) - cal_LineVel_.vel_y * sinf(yaw), 
                                                                   cal_LineVel_.vel_y * cosf(yaw) + cal_LineVel_.vel_x * sinf(yaw), 0));
                    parent_->SetAngularVel(ignition::math::Vector3d(0, 0, cal_LineVel_.vel_th));
                }

                if(publishOdomentry_)
                {
                    publishOdometry(seconds_since_last_update);
                }

                last_odom_publish_time_ = current_time;
            }
        }
    }
    
    // Finalize the controller
    void GazeboRosMecanumDrive::FiniChild()
    {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        rosnode_->shutdown();
        callback_queue_thread_.join();
    }
    
    void GazeboRosMecanumDrive::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
    {
        boost::mutex::scoped_lock scoped_lock(lock);
        x_ = cmd_msg->linear.x;
        y_ = cmd_msg->linear.y;
        rot_ = cmd_msg->angular.z;
    }

    void GazeboRosMecanumDrive::getWheelVelocity()
    {
        wheel_speed_[LEFT_REAR] = (x_ + y_ - rot_ * (wheel_separation_w + wheel_separation_l) / 2.0) / wheel_diameter_ * 2;
        wheel_speed_[LEFT_FRONT] = (x_ - y_ - rot_ * (wheel_separation_w + wheel_separation_l) / 2.0) / wheel_diameter_ * 2;
        wheel_speed_[RIGHT_FRONT] = (x_ + y_ + rot_ * (wheel_separation_w + wheel_separation_l) / 2.0) / wheel_diameter_ * 2;
        wheel_speed_[RIGHT_REAR] = (x_ - y_ + rot_ * (wheel_separation_w + wheel_separation_l) / 2.0) / wheel_diameter_ * 2;
    }
    
    void GazeboRosMecanumDrive::QueueThread()
    {
        static const double timeout = 0.01;
        while (alive_ && rosnode_->ok())
        {
            queue_.callAvailable(ros::WallDuration(timeout));
        }
    }
    
    void GazeboRosMecanumDrive::publishOdometry(double step_time)
    {
        ros::Time current_time = ros::Time::now();
        std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
        std::string base_footprint_frame = tf::resolve(tf_prefix_, robot_base_frame_);

    // getting data for base_footprint to odom transform
    #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = this->parent_->WorldPose();
    #else
        ignition::math::Pose3d pose = this->parent_->GetWorldPose().Ign();
    #endif

        tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
        tf::Vector3    vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

        tf::Transform base_footprint_to_odom(qt, vt);
        transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame, base_footprint_frame));

        // publish odom topic
        odom_.pose.pose.position.x = pose.Pos().X();
        odom_.pose.pose.position.y = pose.Pos().Y();
        
        odom_.pose.pose.orientation.x = pose.Rot().X();
        odom_.pose.pose.orientation.y = pose.Rot().Y();
        odom_.pose.pose.orientation.z = pose.Rot().Z();
        odom_.pose.pose.orientation.w = pose.Rot().W();
        odom_.pose.covariance[0] = 0.00001;
        odom_.pose.covariance[7] = 0.00001;
        odom_.pose.covariance[14] = 1000000000000.0;
        odom_.pose.covariance[21] = 1000000000000.0;
        odom_.pose.covariance[28] = 1000000000000.0;
        odom_.pose.covariance[35] = 0.001;
        
        // get velocity in /odom frame
        ignition::math::Vector3d linear;
        linear.X() = (pose.Pos().X() - last_odom_pose_.Pos().X()) / step_time;
        linear.Y() = (pose.Pos().Y() - last_odom_pose_.Pos().Y()) / step_time;
        if(rot_ > M_PI / step_time)
        {
            // we cannot calculate the angular velocity correctly
            odom_.twist.twist.angular.z = rot_;
        }
        else
        {
            float last_yaw = last_odom_pose_.Rot().Yaw();
            float current_yaw = pose.Rot().Yaw();
            while(current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
            while(current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
            float angular_diff = current_yaw - last_yaw;
            odom_.twist.twist.angular.z = angular_diff / step_time;
        }
        
        last_odom_pose_ = pose;
        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
        odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
        odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
        
        odom_.header.stamp = current_time;
        odom_.header.frame_id = odom_frame;
        odom_.child_frame_id = base_footprint_frame;
        odometry_pub_.publish(odom_);
    }

    void GazeboRosMecanumDrive::calkinematics(linear_vel &line_vel)
    {
        double wheel_encoder[4];
        wheel_encoder[LEFT_REAR] = joints_[LEFT_REAR]->GetVelocity(0);
        wheel_encoder[LEFT_FRONT] = joints_[LEFT_FRONT]->GetVelocity(0);
        wheel_encoder[RIGHT_FRONT] = joints_[RIGHT_FRONT]->GetVelocity(0);
        wheel_encoder[RIGHT_REAR] = joints_[RIGHT_REAR]->GetVelocity(0);

        double l = 1.0 / (2 * (wheel_separation_w + wheel_separation_l));

        line_vel.vel_x = (wheel_encoder[LEFT_REAR] + wheel_encoder[LEFT_FRONT] + wheel_encoder[RIGHT_FRONT] + wheel_encoder[RIGHT_REAR]) * wheel_diameter_ / 2;
        line_vel.vel_y = (wheel_encoder[LEFT_REAR] - wheel_encoder[LEFT_FRONT] + wheel_encoder[RIGHT_FRONT] - wheel_encoder[RIGHT_REAR]) * wheel_diameter_ / 2;
        line_vel.vel_th = (-wheel_encoder[LEFT_REAR] - wheel_encoder[LEFT_FRONT] + wheel_encoder[RIGHT_FRONT] + wheel_encoder[RIGHT_REAR]) * l * wheel_diameter_ / 2;
    }
    
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosMecanumDrive)
}