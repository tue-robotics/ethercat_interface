#include "ros/ros.h"
#include "ethercat_interface/interface.h"
#include "ethercat_interface/exceptions.h"

int main(int argc, char** argv)
{
  // Init rosnode
  ros::init(argc, argv, "test_ai_node");

  // Configure logging
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Get interface name, slave index and parameter index from the parameter server
  std::string ifname;
  int slave_idx = 1;
  int channel_idx = 0;
  ros::NodeHandle nh("~");
  nh.param<std::string>("ifname", ifname, "eth0");
  nh.param<int>("slave", slave_idx, 1);
  nh.param<int>("channel", channel_idx, 0);

  try
  {
    // Construct and initalize the interface
    ROS_INFO("Constructing EtherCAT Interface");
    ethercat_interface::Interface interface(ifname);

    // Test input
    ethercat_interface::InputPtr ai = interface.getSlave(slave_idx).getInput(channel_idx);

    ROS_INFO("Starting loop");
    ros::Rate rate(1000.0);
    while (ros::ok())
    {
      interface.read();

      double value = ai->read();
      ROS_INFO_THROTTLE(1.0, "Read value: %.2f", value);

      interface.write();

      rate.sleep();
    }
  }
  catch (ethercat_interface::SocketException)
  {
    ROS_FATAL("No socket connection on %s. Try excecuting the following "
              "command: sudo setcap cap_net_raw+ep $(readlink $(catkin_find "
              "ethercat_interface test_ai))\n",
              ifname.c_str());
    return 1;
  }

  ROS_INFO("Shutting down");
  return 0;
}
