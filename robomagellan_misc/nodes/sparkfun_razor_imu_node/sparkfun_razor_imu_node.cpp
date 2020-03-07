#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <boost/algorithm/string.hpp>

using namespace std::string_literals;

/**
 * Converts degrees per second to radians per second
 * @param dps An angular velocity in degrees per second
 * @return The angular velocity in radians per second
 */
constexpr double dpsToRps(double dps)
{
    return dps / 57.2958;
}

/**
 * Converts g-unit accelerations to meters per seconds squared
 * @param g A linear acceleration in g-units
 * @return The linear acceleration in meters per seconds squared
 */
constexpr double gToMpss(double g)
{
    return g * 9.80665;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sparkfun_razor_imu_node");

    ros::NodeHandle node_handle;

    const auto port_name = node_handle.param("port", "/dev/ttyACM1"s);
    const auto baud = node_handle.param("baud", 115200);
    const auto frame_id = node_handle.param("frame_id", "imu"s);

    serial::Serial port(port_name, baud);

    if(!port.isOpen())
    {
        ROS_FATAL_STREAM("Unable to open serial port " << port_name);
        return 1;
    }

    auto serial_timeout = serial::Timeout::simpleTimeout(250);
    port.setTimeout(serial_timeout);

    ros::Publisher imu_publisher = node_handle.advertise<sensor_msgs::Imu>("/imu/data", 1);
    ros::Publisher mag_publisher = node_handle.advertise<sensor_msgs::MagneticField>("/imu/mag", 1);

    std_msgs::Header::_seq_type seq = 0;

    while(ros::ok())
    {
        if(!port.waitReadable())
        {
            ROS_WARN("Serial connection timed out without new data available");
            continue;
        }

        const auto serial_message = port.readline();

        std::vector<std::string> tokens;
        boost::split(tokens, serial_message, boost::is_any_of(","));

        if(tokens.size() != 13)
        {
            ROS_WARN_STREAM("Serial connection received a message with the wrong number of tokens (" << tokens.size() << "): " << serial_message);
            continue;
        }

        sensor_msgs::Imu imu_message;
        sensor_msgs::MagneticField mag_message;

        imu_message.header.stamp = ros::Time::now();
        imu_message.header.frame_id = frame_id;
        imu_message.header.seq = seq++;

        mag_message.header = imu_message.header;

        imu_message.linear_acceleration.x = gToMpss(std::stod(tokens[0]));
        imu_message.linear_acceleration.y = gToMpss(std::stod(tokens[1]));
        imu_message.linear_acceleration.z = gToMpss(std::stod(tokens[2]));

        imu_message.angular_velocity.x = dpsToRps(std::stod(tokens[3]));
        imu_message.angular_velocity.y = dpsToRps(std::stod(tokens[4]));
        imu_message.angular_velocity.z = dpsToRps(std::stod(tokens[5]));

        mag_message.magnetic_field.x = std::stod(tokens[6]);
        mag_message.magnetic_field.y = std::stod(tokens[7]);
        mag_message.magnetic_field.z = std::stod(tokens[8]);

        imu_message.orientation.w = std::stod(tokens[9]);
        imu_message.orientation.x = std::stod(tokens[10]);
        imu_message.orientation.y = std::stod(tokens[11]);
        imu_message.orientation.z = std::stod(tokens[12]);

        imu_message.orientation_covariance[0] = -1;
        imu_message.linear_acceleration_covariance[0] = -1;
        imu_message.angular_velocity_covariance[0] = -1;

        imu_publisher.publish(imu_message);
        mag_publisher.publish(mag_message);

        ros::spinOnce();
    }

    return 0;
}