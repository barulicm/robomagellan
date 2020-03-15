#include "robomagellan_hardware_interface/robomagellan_hardware_interface.h"
#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robomagellan_hardware_interface");
    ros::CallbackQueue callback_queue;
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");
    node_handle.setCallbackQueue(&callback_queue);
    robomagellan_hardware_interface::RobomagellanHardwareInterface hardware_interface(node_handle, private_node_handle);
    ros::MultiThreadedSpinner spinner;
    spinner.spin(&callback_queue);
    return 0;
}