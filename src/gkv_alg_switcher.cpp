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
#include <GKV_Device.h>

using namespace std;

uint8_t algorithm = GKV_ADC_CODES_ALGORITHM;

void AdcCallback(const gkv_ros_driver::GkvAdcData::ConstPtr& msg);

void SensorsCallback(const gkv_ros_driver::GkvSensorsData::ConstPtr& msg);

void GyrovertCallback(const gkv_ros_driver::GkvGyrovertData::ConstPtr& msg);

void InclinometerCallback(const gkv_ros_driver::GkvInclinometerData::ConstPtr& msg);

void BINSCallback(const gkv_ros_driver::GkvBINSData::ConstPtr& msg);

void GNSSCallback(const gkv_ros_driver::GkvGpsData::ConstPtr& msg);

void ExtGNSSCallback(const gkv_ros_driver::GkvExtGpsData::ConstPtr& msg);

void CustomDataCallback(const gkv_ros_driver::GkvCustomData::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gkv_alg_switcher");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  // DEFINE SUBSCRIBERS FOR EVERY DATA PACKET
  ros::Subscriber sub_adc = n.subscribe("gkv_adc_data", 1000, AdcCallback);
  ros::Subscriber sub_raw = n.subscribe("gkv_sensors_data", 1000, SensorsCallback);
  ros::Subscriber sub_gyrovert = n.subscribe("gkv_gyrovert_data", 1000, GyrovertCallback);
  ros::Subscriber sub_incl = n.subscribe("gkv_inclinometer_data", 1000, InclinometerCallback);
  ros::Subscriber sub_bins = n.subscribe("gkv_bins_data", 1000, BINSCallback);
  ros::Subscriber sub_gnss = n.subscribe("gkv_gnss_data", 1000, GNSSCallback);
  ros::Subscriber sub_ext_gnss = n.subscribe("gkv_ext_gnss_data", 1000, ExtGNSSCallback);
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

  // DEFINE CLIENT FOR RESET SERVICE
  ros::ServiceClient reset_client = n.serviceClient<gkv_ros_driver::GkvReset>("gkv_reset_srv");
  gkv_ros_driver::GkvReset reset_srv;
   if (reset_client.call(reset_srv))
   {
         ROS_INFO("Reseted: %d", (int)reset_srv.response.result);
   }
   else
   {
        ROS_ERROR("Failed to reset device");
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
    set_packet_type_srv.request.packet_type = GKV_SELECT_DEFAULT_ALGORITHM_PACKET;
     if (set_packet_type_client.call(set_packet_type_srv))
     {
       ROS_INFO("Packet type changed: %d", (int)set_packet_type_srv.response.result);
     }
     else
     {
       ROS_ERROR("Failed to set packet type");
     }

    // DEFINE CLIENT FOR ID REQUEST
    ros::ServiceClient get_id_client = n.serviceClient<gkv_ros_driver::GkvGetID>("gkv_get_id_srv");
    gkv_ros_driver::GkvGetID get_id_srv;
     if (get_id_client.call(get_id_srv))
     {
        ROS_INFO("Description: [%s]", (get_id_srv.response.dev_description.data).c_str());
        ROS_INFO("ID: [%s]", (get_id_srv.response.dev_id.data).c_str());
     }
     else
     {
          ROS_ERROR("Failed to get ID");
          ROS_INFO("Description: [%s]", (get_id_srv.response.dev_description.data).c_str());
          ROS_INFO("ID: [%s]", (get_id_srv.response.dev_id.data).c_str());
     }
     ros::waitForShutdown();
  return 0;
}

void AdcCallback(const gkv_ros_driver::GkvAdcData::ConstPtr& msg)
{
  ROS_INFO("Accelerometers: ax=[%d] ay=[%d] az=[%d]", msg->acceleration_adc_x, msg->acceleration_adc_y, msg->acceleration_adc_z);
  ROS_INFO("Gyroscopes: wx=[%d] wy=[%d] wz=[%d]", msg->angular_rate_adc_x, msg->angular_rate_adc_y, msg->angular_rate_adc_z);
}

void SensorsCallback(const gkv_ros_driver::GkvSensorsData::ConstPtr& msg)
{
  ROS_INFO("Accelerometers: [%f] [%f] [%f]", msg->acceleration_x, msg->acceleration_y, msg->acceleration_z);
  ROS_INFO("Gyroscopes: [%f] [%f] [%f]", msg->angular_rate_x, msg->angular_rate_y, msg->angular_rate_z);
}

void GyrovertCallback(const gkv_ros_driver::GkvGyrovertData::ConstPtr& msg)
{
  ROS_INFO("Orientation: yaw=[%f] pitch=[%f] roll=[%f]", msg->yaw, msg->pitch, msg->roll);
}

void InclinometerCallback(const gkv_ros_driver::GkvInclinometerData::ConstPtr& msg)
{
  ROS_INFO("Inclination: alpha=[%f] beta=[%f]", msg->alpha, msg->beta);
}

void BINSCallback(const gkv_ros_driver::GkvBINSData::ConstPtr& msg)
{
  ROS_INFO("BINS Data: time=[%f] latitude=[%f] longitude=[%f] yaw=[%f] pitch=[%f] roll=[%f] alpha=[%f] beta=[%f] q0=[%f] q1=[%f] q2=[%f] q3=[%f]",
                  msg->x, msg->y, msg->z, msg->yaw, msg->pitch, msg->roll, msg->alpha, msg->beta, msg->q0, msg->q1, msg->q2, msg->q3);
}

void GNSSCallback(const gkv_ros_driver::GkvGpsData::ConstPtr& msg)
{
  ROS_INFO("GNSS Data: time=[%f] latitude=[%f] longitude=[%f] altitude=[%f] gps_state_status=[%d] TDOP=[%f] HDOP=[%f] VDOP=[%f] horizontal_vel=[%f] azimuth=[%f] vertical_vel=[%f]",
                  msg->time, msg->latitude, msg->longitude, msg->altitude, msg->gps_state_status, msg->TDOP, msg->HDOP, msg->VDOP, msg->horizontal_vel, msg->azimuth, msg->vertical_vel);
}

void ExtGNSSCallback(const gkv_ros_driver::GkvExtGpsData::ConstPtr& msg)
{
  ROS_INFO("Extended GNSS Data: lat_vel=[%f] lon_vel=[%f] lat_std=[%f] lon_std=[%f] alt_std=[%f] lat_vel_std=[%f] lon_vel_std=[%f] alt_vel_std=[%f] num_ss=[%d]",
                  msg->lat_vel, msg->lon_vel, msg->lat_std, msg->lon_std, msg->alt_std, msg->lat_vel_std, msg->lon_vel_std, msg->alt_vel_std, msg->num_ss);
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
