#include "ros/ros.h"
#include "ethercat_interface/ethercat_interface.h"
#include "ethercat_interface/exceptions.h"

int main(int argc, char** argv)
{
    // Init rosnode
    ros::init(argc, argv, "test_do_node");

    // Configure logging
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
       ros::console::notifyLoggerLevelsChanged();
    }

    // Get interface name, slave index and parameter index from the parameter server
    std::string ifname;
    int slave_idx = 4;
    int channel_idx = 0;;
    ros::NodeHandle nh("~");
    nh.param<std::string>("ifname", ifname, "eth0");
    nh.param<int>("slave", slave_idx, 4);
    nh.param<int>("channel", channel_idx, 0);
    ROS_INFO("Using slave %i, channel %i", slave_idx, channel_idx);

    // Construct and initalize the interface
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
                  "ethercat_interface test_do))\n",
                  ifname.c_str());
        exit(1);
    }

    // Test output
    double value = 0.0;
    std::shared_ptr<WriteInterface> do_ = interface.getSlave(slave_idx).getOutput(channel_idx);
    do_->write(value);

    ROS_INFO("Starting loop");
    ros::Rate rate(1000.0);
    int count = 0;
    while (ros::ok())
    {
        interface.receiveAll();

        count += 1;
        if (count == 2000)
        {
            if (value > 0.5)
            {
                value = 0.0;
            }
            else
            {
                value = 1.0;
            }
            ROS_INFO("Setting output of slave %u, channel %u to %.2f", slave_idx, channel_idx, value);
            do_->write(value);
            count = 0;
        }
//        do_->write(value);

        interface.sendAll();


        rate.sleep();
    }

    ROS_INFO("Shutting down");
    return 0;
}
