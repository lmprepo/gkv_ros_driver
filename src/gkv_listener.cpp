#include "ros/ros.h"
#include <gkv_ros_driver/GkvAdcData.h>
#include <gkv_ros_driver/GkvSensorsData.h>
#include <gkv_ros_driver/GkvGyrovertData.h>
#include <gkv_ros_driver/GkvInclinometerData.h>
#include <gkv_ros_driver/GkvBINSData.h>
#include <gkv_ros_driver/GkvGpsData.h>
#include <gkv_ros_driver/GkvExtGpsData.h>
#include <gkv_ros_driver/GkvCustomData.h>
#include <gkv_ros_driver/GkvReset.h>
#include <gkv_ros_driver/GkvSetAlgorithm.h>
#include <GKV_Device.h>

using namespace std;

uint8_t selected_algorithm = 0;

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gkv_listener");

  ros::NodeHandle n;
  // DEFINE SUBSCRIBERS FOR EVERY DATA PACKET
  ros::Subscriber sub_adc = n.subscribe("gkv_adc_data", 1000, AdcCallback);
  ros::Subscriber sub_raw = n.subscribe("gkv_sensors_data", 1000, SensorsCallback);
  ros::Subscriber sub_gyrovert = n.subscribe("gkv_gyrovert_data", 1000, GyrovertCallback);
  ros::Subscriber sub_incl = n.subscribe("gkv_inclinometer_data", 1000, InclinometerCallback);
  ros::Subscriber sub_bins = n.subscribe("gkv_bins_data", 1000, BINSCallback);
  ros::Subscriber sub_gnss = n.subscribe("gkv_gnss_data", 1000, GNSSCallback);
  ros::Subscriber sub_ext_gnss = n.subscribe("gkv_ext_gnss_data", 1000, ExtGNSSCallback);
  ros::Subscriber sub_custom = n.subscribe("gkv_custom_data", 1000, CustomDataCallback);
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
        return 1;
   }
   // DEFINE CLIENT FOR SET ALGORITHM
   ros::ServiceClient set_alg_client = n.serviceClient<gkv_ros_driver::GkvSetAlgorithm>("gkv_set_alg_srv");
   gkv_ros_driver::GkvSetAlgorithm set_alg_srv;
   set_alg_srv.request.algorithm_number = selected_algorithm;
    if (set_alg_client.call(set_alg_srv))
    {
      ROS_INFO("Algortithm changed: %d", (int)set_alg_srv.response.result);
    }
    else
    {
     ROS_ERROR("Failed to set algorithm");
     return 1;
    }
  ros::spin();
  return 0;
}
