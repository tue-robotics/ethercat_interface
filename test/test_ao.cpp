#include "ros/ros.h"
#include "ethercat_interface/ethercat_interface.h"
#include "ethercat_interface/exceptions.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ethercat_interface_node");

    // ToDo: add command line arguments for slave and channel indices
    unsigned int slave_idx = 1;
    unsigned int channel_idx = 0;

    ros::NodeHandle nh("~");

    std::string ifname;
    nh.param<std::string>("ifname", ifname, "eth0");

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
                  "ethercat_interface test_ao))\n",
                  ifname.c_str());
        exit(1);
    }

    // Test output
    double value = -2.0;
    std::shared_ptr<IOInterface> ao = interface.getInterface(slave_idx, channel_idx);
    ao->write(value);

    ROS_INFO("Starting loop");
    ros::Rate rate(1000.0);
    int count = 0;
    while (ros::ok())
    {
        interface.receiveAll();

        count += 1;
        if (count == 2000)
        {
            value += 0.5;
            if (value > 2.01)
            {
                value = -2.0;
            }
            ROS_INFO("Setting output of slave %u, channel %u to %.2f", slave_idx, channel_idx, value);
            ao->write(value);
            count = 0;
        }

        interface.sendAll();


        rate.sleep();
    }

    ROS_INFO("Shutting down");
    return 0;
}
