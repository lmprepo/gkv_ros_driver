#include "ros/ros.h"
#include "std_msgs/String.h"
#include <gkv_ros_driver/GkvAdcData.h>
#include <gkv_ros_driver/GkvSensorsData.h>
#include <gkv_ros_driver/GkvGyrovertData.h>
#include <gkv_ros_driver/GkvInclinometerData.h>
#include <gkv_ros_driver/GkvBINSData.h>
#include <gkv_ros_driver/GkvGpsData.h>
#include <gkv_ros_driver/GkvExtGpsData.h>
#include <gkv_ros_driver/GkvCustomData.h>
#include <gkv_ros_driver/GkvCheckConnection.h>
#include <gkv_ros_driver/GkvReset.h>
#include <gkv_ros_driver/GkvSetAlgorithm.h>
#include <gkv_ros_driver/GkvGetID.h>
#include <gkv_ros_driver/GkvSetPacketType.h>
#include <gkv_ros_driver/GkvSetCustomParameters.h>
#include <GKV_Device.h>

using namespace std;

uint8_t algorithm = GKV_ORIENTATION_KALMAN_ALGORITHM;

void CustomDataCallback(const gkv_ros_driver::GkvCustomData::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gkv_custom_packet_listener");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  // DEFINE SUBSCRIBERS FOR EVERY DATA PACKET
  ros::Subscriber sub_custom = n.subscribe("gkv_custom_data", 1000, CustomDataCallback);

  // DEFINE CLIENT FOR CHECK CONNECTION SERVICE
  ros::ServiceClient check_connection_client = n.serviceClient<gkv_ros_driver::GkvCheckConnection>("gkv_check_connection_srv");
  gkv_ros_driver::GkvCheckConnection check_connection_srv;
   if (check_connection_client.call(check_connection_srv))
   {
         ROS_INFO("Connection state: %d", (int)check_connection_srv.response.result);
   }
   else
   {
         ROS_ERROR("Connection state: %d", (int)check_connection_srv.response.result);
         return 1;
   }

   // DEFINE CLIENT FOR ALGORITHM SETTING
   ros::ServiceClient set_alg_client = n.serviceClient<gkv_ros_driver::GkvSetAlgorithm>("gkv_set_alg_srv");
   gkv_ros_driver::GkvSetAlgorithm set_alg_srv;
   set_alg_srv.request.algorithm_number = algorithm;
    if (set_alg_client.call(set_alg_srv))
    {
      ROS_INFO("Algortithm changed: %d", (int)set_alg_srv.response.result);
    }
    else
    {
      ROS_ERROR("Failed to set algorithm");
    }
    // DEFINE CLIENT FOR PACKET TYPE SETTING
    ros::ServiceClient set_packet_type_client = n.serviceClient<gkv_ros_driver::GkvSetPacketType>("gkv_set_packet_type_srv");
    gkv_ros_driver::GkvSetPacketType set_packet_type_srv;
    set_packet_type_srv.request.packet_type = GKV_SELECT_CUSTOM_PACKET;
     if (set_packet_type_client.call(set_packet_type_srv))
     {
       ROS_INFO("Packet type changed: %d", (int)set_packet_type_srv.response.result);
     }
     else
     {
       ROS_ERROR("Failed to set packet type");
     }

     // DEFINE CLIENT FOR CUSTOM PARAMETERS SETTING
     ros::ServiceClient set_custom_params_client = n.serviceClient<gkv_ros_driver::GkvSetCustomParameters>("gkv_set_custom_params_srv");
     gkv_ros_driver::GkvSetCustomParameters set_custom_params_srv;
     set_custom_params_srv.request.params={GKV_YAW, GKV_PITCH, GKV_ROLL, GKV_Q0, GKV_Q1, GKV_Q2, GKV_Q3};
     set_custom_params_srv.request.quantity_of_params = set_custom_params_srv.request.params.size();
      if (set_custom_params_client.call(set_custom_params_srv))
      {
        ROS_INFO("Custom parameters changed: %d", (int)set_custom_params_srv.response.result);
      }
      else
      {
        ROS_ERROR("Failed to set parameters");
      }
     ros::waitForShutdown();
  return 0;
}


void CustomDataCallback(const gkv_ros_driver::GkvCustomData::ConstPtr& msg)
{
    char str[30];
    ROS_INFO("CustomPacket: [%d]", msg->quantity_of_params);

    for (uint8_t i = 0; i < msg->quantity_of_params; i++)
    {
        if (msg->param_values[i] == msg->param_values[i])// проверка на isnan
        {
            cout << "param = ["<< to_string(msg->numbers[i]) << "] " << to_string(msg->param_values[i]) << ' ';
        }
        else
        {
            cout << "param = NaN ";
        }
    }
    cout << endl;
}
