#include "ros/ros.h"
#include "ethercat_interface/interface.h"
#include "ethercat_interface/exceptions.h"

int main(int argc, char** argv)
{
  // Init rosnode
  ros::init(argc, argv, "list_io");

  // Configure logging
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Get interface name, slave index and parameter index from the parameter server
  std::string ifname;
  ros::NodeHandle nh("~");
  nh.param<std::string>("ifname", ifname, "eth0");

  try
  {
    // Construct and initalize the interface
    ROS_INFO("Constructing EtherCAT Interface");
    ethercat_interface::Interface interface(ifname);
  }
  catch (ethercat_interface::SocketException)
  {
    ROS_FATAL("No socket connection on %s. Try excecuting the following "
              "command: sudo setcap cap_net_raw+ep $(readlink $(catkin_find "
              "ethercat_interface list_interfaces))\n",
              ifname.c_str());
    return 1;
  }

  ROS_INFO("Shutting down");
  return 0;
}
