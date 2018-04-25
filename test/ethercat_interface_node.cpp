#include "ros/ros.h"
#include "ethercat_interface/interface.h"
#include "ethercat_interface/exceptions.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ethercat_interface_node");

  ros::NodeHandle nh("~");

  std::string ifname;
  nh.param<std::string>("ifname", ifname, "eth0");

  try
  {
    ROS_INFO("Constructing EtherCAT Interface");
    ethercat_interface::Interface interface(ifname);

    // Test output
    double value = -2.0;
    ethercat_interface::OutputPtr ao0 = interface.getSlave(1).getOutput(0);
    // TODO: if an interface does not exist, raise an exception instead of segfaulting
    ao0->write(value);

    ROS_INFO("Getting encoder");
    ethercat_interface::InputPtr encoder = interface.getSlave(2).getInput(0);
    ROS_INFO("Got encoder");

    ROS_INFO("Starting loop");
    ros::Rate rate(1000.0);
    int count = 0;
    while (ros::ok())
    {
      interface.read();

      ROS_INFO("Encoder: %.2f", encoder->read());

      count += 1;
      if (count == 1000)
      {
        value *= -1.0;
        ao0->write(value);
        //            ao1->write(2.0 * value);
        count = 0;
      }

      interface.write();

      rate.sleep();
    }
  }
  catch (ethercat_interface::SocketException)
  {
    ROS_FATAL("No socket connection on %s. Try excecuting the following "
              "command: sudo setcap cap_net_raw+ep $(readlink $(catkin_find "
              "ethercat_interface ethercat_interface_node))\n",
              ifname.c_str());
    return 1;
  }

  ROS_INFO("Shutting down");
  return 0;
}
