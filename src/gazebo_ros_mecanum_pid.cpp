#include <gazebo_mecanum_plugins/gazebo_ros_mecanum_pid.h>

namespace gazebo
{
    pid_controller::pid_controller(physics::ModelPtr model)
    {
        mobile_model = model;

        max_value_ = 10;
        min_value_ = -10;
    }

    pid_controller::~pid_controller()
    {

    }

    void pid_controller::gazebo_PID_LEFT_REAR_(physics::JointPtr joint, double vel)
    {
        LEFT_REAR_pid = common::PID(LEFT_REAR_.P, LEFT_REAR_.I, LEFT_REAR_.D);
        mobile_model->GetJointController()->SetVelocityPID(joint->GetScopedName(), LEFT_REAR_pid);
        mobile_model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), vel);
    }

    void pid_controller::gazebo_PID_LEFT_FRONT_(physics::JointPtr joint, double vel)
    {
        LEFT_FRONT_pid = common::PID(LEFT_FRONT_.P, LEFT_FRONT_.I, LEFT_FRONT_.D);
        mobile_model->GetJointController()->SetVelocityPID(joint->GetScopedName(), LEFT_FRONT_pid);
        mobile_model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), vel);
    }

    void pid_controller::gazebo_PID_RIGHT_FRONT_(physics::JointPtr joint, double vel)
    {
        RIGHT_FRONT_pid = common::PID(RIGHT_FRONT_.P, RIGHT_FRONT_.I, RIGHT_FRONT_.D);
        mobile_model->GetJointController()->SetVelocityPID(joint->GetScopedName(), RIGHT_FRONT_pid);
        mobile_model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), vel);
    }

    void pid_controller::gazebo_PID_RIGHT_REAR_(physics::JointPtr joint, double vel)
    {
        RIGHT_REAR_pid = common::PID(RIGHT_REAR_.P, RIGHT_REAR_.I, RIGHT_REAR_.D);
        mobile_model->GetJointController()->SetVelocityPID(joint->GetScopedName(), RIGHT_REAR_pid);
        mobile_model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), vel);
    }
}