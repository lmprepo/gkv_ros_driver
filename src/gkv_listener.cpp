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
#include <gkv_ros_driver/GkvReset.h>
#include <gkv_ros_driver/GkvSetAlgorithm.h>
#include <gkv_ros_driver/GkvGetID.h>
#include <geometry_msgs/PoseStamped.h>
#include <GKV_Device.h>

using namespace std;

void AdcCallback(const gkv_ros_driver::GkvAdcData::ConstPtr& msg);

void SensorsCallback(const gkv_ros_driver::GkvSensorsData::ConstPtr& msg);

void GyrovertCallback(const gkv_ros_driver::GkvGyrovertData::ConstPtr& msg);

void InclinometerCallback(const gkv_ros_driver::GkvInclinometerData::ConstPtr& msg);

void BINSCallback(const gkv_ros_driver::GkvBINSData::ConstPtr& msg);

void GNSSCallback(const gkv_ros_driver::GkvGpsData::ConstPtr& msg);

void ExtGNSSCallback(const gkv_ros_driver::GkvExtGpsData::ConstPtr& msg);

void CustomDataCallback(const gkv_ros_driver::GkvCustomData::ConstPtr& msg);

void PoseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gkv_listener");
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
  ros::Subscriber sub_pose_stamped = n.subscribe("gkv_pose_stamped_data", 1000, PoseStampedCallback);
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


void PoseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("Pose Stamped Data: x=[%f] y=[%f] z=[%f] q0(w)=[%f] q1(x)=[%f] q2(y)=[%f] q3(z)=[%f]",
                  msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}
