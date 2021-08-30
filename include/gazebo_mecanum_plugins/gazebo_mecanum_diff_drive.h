#ifndef GAZEBO_PLUGINS_GAZEBO_ROS_MECANUM_DIFF_DRIVE_HPP_
#define GAZEBO_PLUGINS_GAZEBO_ROS_MECANUM_DIFF_DRIVE_HPP_

#include <gazebo_mecanum_plugins/gazebo_mecanum_diff_drive_private.h>
#include <gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_mecanum_plugins
{
  class GazeboMecanumRosDiffDrive : public gazebo::ModelPlugin
  {
    public:
      /// Constructor
      GazeboMecanumRosDiffDrive();
    
      /// Destructor
      ~GazeboMecanumRosDiffDrive();
    
    protected:
      // Documentation inherited
      void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

      // Documentation inherited
      void Reset() override;

    private:
    /// Private data pointer
    std::unique_ptr<GazeboMecanumRosDiffDrivePrivate> impl_;
  };
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_DIFF_DRIVE_HPP_