#include "ros/ros.h"
#include "ethercat_interface/ethercat_interface.h"
#include "ethercat_interface/exceptions.h"

int main(int argc, char** argv)
{
    // Init rosnode
    ros::init(argc, argv, "test_encoder_node");

    // Get interface name, slave index and parameter index from the parameter server
    std::string ifname;
    int slave_idx = 1;
    int channel_idx = 0;
    ros::NodeHandle nh("~");
    nh.param<std::string>("ifname", ifname, "eth0");
    nh.param<int>("slave", slave_idx, 2);
    nh.param<int>("channel", channel_idx, 0);

    ROS_INFO("Constructing EtherCAT Interface");
    EthercatInterface interface;

    ROS_INFO("Initializing");
    try
    {
        if (!interface.initialize(ifname))
        {
            ROS_ERROR("Something went terribly wrong...");
            exit(1);
        }
    }
    catch (SocketError)
    {
        ROS_ERROR("No socket connection on %s. Try excecuting the following "
                  "command: sudo setcap cap_net_raw+ep $(readlink $(catkin_find "
                  "ethercat_interface ethercat_interface_node))\n",
                  ifname.c_str());
        exit(1);
    }

    // Get encoder interface
    ROS_INFO("Getting encoder");
    std::shared_ptr<IOInterface> encoder = interface.getInterface(2, 0);
    ROS_INFO("Got encoder");

    ROS_INFO("Starting loop");
    ros::Rate rate(1000.0);
    while (ros::ok())
    {
        interface.receiveAll();

        ROS_INFO_THROTTLE(1.0, "Encoder: %i", encoder->read());

        interface.sendAll();

        rate.sleep();
    }

    ROS_INFO("Shutting down");
    return 0;
}
