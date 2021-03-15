#include "ros/ros.h"
#include "GKV_DeviceROSWrapper.h"


int main(int argc, char **argv)
{
    std::string com_port;
    std::cout << "Set Serial Port:";
    std::cin >> com_port;
    std::cout << "#start connecting to " << com_port << "\n";
    ros::init(argc, argv, "gkv_controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    GKV_DeviceROSWrapper * gkv= new GKV_DeviceROSWrapper(&nh,com_port,B921600, GKV_ROS_PACKET_MODE);
    if (gkv->IsConnected())
    {
        ROS_INFO("GKV driver is now started");
    }
    else {
        ROS_ERROR("Failed to connect device");
        return 0;
    }
    ros::waitForShutdown();
}
// %EndTag(FULLTEXT)%
