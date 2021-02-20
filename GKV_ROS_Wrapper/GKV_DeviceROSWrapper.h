#ifndef GKV_DEVICE_ROS_WRAPPER_H
#define GKV_DEVICE_ROS_WRAPPER_H
#include "ros/ros.h"
#include <string.h>
#include <unistd.h>
#include "GKV_Device.h"
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
#include "std_msgs/String.h"
//using namespace ;

class GKV_DeviceROSWrapper
{
private:
    Gyrovert::GKV_Device* gkv_;
    ros::Publisher received_adc_data_publisher;
    ros::Publisher received_sensor_data_publisher;
    ros::Publisher received_gyrovert_data_publisher;
    ros::Publisher received_inclinometer_data_publisher;
    ros::Publisher received_bins_data_publisher;
    ros::Publisher received_gnss_data_publisher;
    ros::Publisher received_ext_gnss_data_publisher;
    ros::Publisher received_custom_data_publisher;

    ros::ServiceServer ResetService;
    ros::ServiceServer SetAlgorithmService;
    ros::ServiceServer GetIDService;

    uint8_t GKV_Status=0;
    uint16_t request_limit=100;
    bool ResetRequestFlag=false;
    bool SetAlgRequestFlag=false;
    bool CustomParamNumbersRequestFlag=false;
    Gyrovert::GKV_CustomDataParam device_custom_parameters;
    bool CustomParamNumbersReceived=false;
    Gyrovert::GKV_ID device_id;
    bool RequestDevIDFlag=false;
    const char* NoDevStr = "NoDeviceFound";
public:
    GKV_DeviceROSWrapper(ros::NodeHandle *nh, std::string serial_port, uint32_t baudrate);
    //RESET DEVICE FUNCTION
    bool ResetDevice(gkv_ros_driver::GkvReset::Request  &req,
                     gkv_ros_driver::GkvReset::Response &res);
    //SET DEVICE ALGORITHM FUNCTION
    bool SetAlgorithm(gkv_ros_driver::GkvSetAlgorithm::Request  &req,
                     gkv_ros_driver::GkvSetAlgorithm::Response &res);
    //GET DEVICE ID FUNCTION
    bool GetID(gkv_ros_driver::GkvGetID::Request  &req,
               gkv_ros_driver::GkvGetID::Response &res);
    //CHECK CONNECTION STATUS FUNCTION
    bool IsConnected()
    {
        return gkv_->GetSerialConnectionState();
    }
    //SEND RECEIVED DATA TO TOPICS FUNCTION
    void publishReceivedData(Gyrovert::GKV_PacketBase * buf);
};
#endif
