#include "ros/ros.h"
#include "ethercat_interface/ethercat_interface.h"
#include "ethercat_interface/exceptions.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ethercat_interface_node");

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
                  "ethercat_interface ethercat_interface_node))\n",
                  ifname.c_str());
        exit(1);
    }

    // Test output
    double value = -2.0;
    std::shared_ptr<IOInterface> ao0 = interface.getInterface(1, 0);
    ao0->write(value);
//    std::shared_ptr<IOInterface> ao1 = interface.getInterface(1, 1);
//    ao1->write(2.0 * value);

//    EtherCatDriver ao_driver = interface.getSlave(1);
////    IOInterface ao1 = ao_driver.getChannel(1);
//    IOInterface ao1 = ao_driver.getChannel(1);

    ROS_INFO("Getting encoder");
    std::shared_ptr<IOInterface> encoder = interface.getInterface(2, 0);
    ROS_INFO("Got encoder");

    ROS_INFO("Starting loop");
    ros::Rate rate(1000.0);
    int count = 0;
    while (ros::ok())
    {
        interface.receiveAll();

        ROS_INFO("Encoder: %i", encoder->read());

        count += 1;
        if (count == 1000)
        {
            value *= -1.0;
            ao0->write(value);
//            ao1->write(2.0 * value);
            count = 0;
        }

        interface.sendAll();


        rate.sleep();
    }

    ROS_INFO("Shutting down");
    return 0;
}
