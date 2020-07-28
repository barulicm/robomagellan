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
#include <serial/serial.h>

namespace robomagellan_hardware_interface
{
class RobomagellanHardwareInterface : public hardware_interface::RobotHW
{
public:
  RobomagellanHardwareInterface(ros::NodeHandle &node_handle, ros::NodeHandle& private_node_handle);
  ~RobomagellanHardwareInterface() = default;

private:
  void setupJoint(const std::string& name, int index);
  void update(const ros::TimerEvent& e);
  void read();
  void write(const ros::Duration& elapsed_time);
  void tryToOpenPort();

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_soft_limits_interface_;

  std::string port_name_;

  std::array<double, 2> joint_positions_ = {};
  std::array<double, 2> joint_velocities_ = {};
  std::array<double, 2> joint_efforts_ = {};
  std::array<double, 2> joint_velocity_commands_ = {};

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Timer update_timer_;
  double loop_hz_ = 10.0;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  ros::Publisher battery_publisher_;

  serial::Serial serial_port_;

  std::string last_sent_message_;
};
}

#endif
