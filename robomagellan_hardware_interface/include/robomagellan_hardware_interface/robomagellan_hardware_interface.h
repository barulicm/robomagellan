#ifndef ROBOMAGELLAN_HARDWARE_INTERFACE_H
#define ROBOMAGELLAN_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

namespace robomagellan_hardware_interface
{
class RobomagellanHardwareInterface : public hardware_interface::RobotHW
{
public:
  RobomagellanHardwareInterface(ros::NodeHandle &node_handle);
  ~RobomagellanHardwareInterface() = default;

  void init();
  void update(const ros::TimerEvent& e);
  void read();
  void write(ros::Duration elapsed_time);

private:
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_soft_limits_interface_;

  int num_joints_;
  std::vector<std::string> joint_names;
  std::array<double, 2> joint_positions_;
  std::array<double, 2> joint_velocities_;
  std::array<double, 2> joint_efforts_;
  std::array<double, 2> joint_velocity_commands_;
  std::array<double, 2> joint_lower_limits_;
  std::array<double, 2> joint_upper_limits_;

  ros::NodeHandle node_handle_;
  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  double loop_hz_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  ros::Publisher battery_publisher_;

  void setupJoint(const std::string& name, int index);
};
}

#endif
