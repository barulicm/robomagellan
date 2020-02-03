#include "robomagellan_hardware_interface/robomagellan_hardware_interface.h"
#include <sensor_msgs/BatteryState.h>
#include <cmath>

namespace robomagellan_hardware_interface
{

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsInterface;

RobomagellanHardwareInterface::RobomagellanHardwareInterface(ros::NodeHandle &node_handle)
    : node_handle_(node_handle)
{
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, node_handle_));
    node_handle_.param("/robomagellan/hardware_interface/loop_hz", loop_hz_, 0.1);
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
    non_realtime_loop_ = node_handle_.createTimer(update_freq, &RobomagellanHardwareInterface::update, this);
}

void RobomagellanHardwareInterface::setupJoint(const std::string& name, int index)
{
    JointStateHandle joint_state_handle(name, &joint_positions_[index], &joint_velocities_[index], &joint_efforts_[index]);
    joint_state_interface_.registerHandle(joint_state_handle);

    JointHandle joint_velocity_handle(joint_state_handle, &joint_velocity_commands_[index]);
    JointLimits limits;
    SoftJointLimits soft_limits;
    getJointLimits(name, node_handle_, limits);
    VelocityJointSoftLimitsHandle joint_limits_handle(joint_velocity_handle, limits, soft_limits);
    velocity_joint_soft_limits_interface_.registerHandle(joint_limits_handle);
    velocity_joint_interface_.registerHandle(joint_velocity_handle);
}

void RobomagellanHardwareInterface::init()
{
    setupJoint("left_wheel", 0);
    setupJoint("right_wheel", 1);

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocity_joint_soft_limits_interface_);

    battery_publisher_ = node_handle_.advertise<sensor_msgs::BatteryState>("/robomagellan/battery", 1);
}

void RobomagellanHardwareInterface::update(const ros::TimerEvent& e)
{
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void RobomagellanHardwareInterface::read()
{


    sensor_msgs::BatteryState battery_msg;
    battery_msg.voltage = 0.0f; // TODO read from serial
    battery_msg.current = NAN;
    battery_msg.charge = NAN;
    battery_msg.capacity = NAN;
    battery_msg.design_capacity = NAN;
    battery_msg.percentage = NAN;
    battery_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    battery_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
    battery_publisher_.publish(battery_msg);
}

constexpr double radPerSecTORPM(double rad_per_sec)
{
    return rad_per_sec * 9.5492965964254;
}

void RobomagellanHardwareInterface::write(ros::Duration elapsed_time)
{
    const auto left_rpm = radPerSecTORPM(joint_velocity_commands_[0]);
    const auto right_rpm = radPerSecTORPM(joint_velocity_commands_[1]);
}
}