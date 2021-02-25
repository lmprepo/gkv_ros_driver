/**
  ******************************************************************************
  * @file    GKV_DeviceROSWrapper.h
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   GKV-10 class module for data receiving and transmitting in ROS
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 Laboratory of Microdevices, Ltd. </center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Laboratory of Microdevices nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

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
#include <gkv_ros_driver/GkvCheckConnection.h>
#include <gkv_ros_driver/GkvSetPacketType.h>
#include <gkv_ros_driver/GkvSetCustomParameters.h>

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
    ros::ServiceServer CheckConnectionService;
    ros::ServiceServer SetPacketTypeService;
    ros::ServiceServer SetCustomParametersService;
    ros::ServiceServer GetDeviceSettingsService;


    uint8_t GKV_Status=0;
    uint16_t request_limit=100;
    uint8_t custom_params_limit=64;
    uint8_t max_number_of_custom_parameter=109;

    bool ResetRequestFlag=false;
    bool SetAlgRequestFlag=false;
    bool CustomParamNumbersRequestFlag=false;
    Gyrovert::GKV_CustomDataParam device_custom_parameters;
    bool CustomParamNumbersReceived=false;
    Gyrovert::GKV_ID device_id;
    bool RequestDevIDFlag=false;
    bool CheckConnectionRequestFlag=false;
    bool SetPacketTypeRequestFlag=false;
    bool SetCustomParametersRequestFlag=false;



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
    //Check DEVICE Connection FUNCTION
    bool CheckConnection(gkv_ros_driver::GkvCheckConnection::Request  &req,
                        gkv_ros_driver::GkvCheckConnection::Response &res);
    //SET DEVICE PACKET TYPE FUNCTION
    bool SetPacketType(gkv_ros_driver::GkvSetPacketType::Request  &req,
                     gkv_ros_driver::GkvSetPacketType::Response &res);
    //SET DEVICE CUSTOM PARAMS FUNCTION
    bool SetCustomParams(gkv_ros_driver::GkvSetCustomParameters::Request  &req,
                     gkv_ros_driver::GkvSetCustomParameters::Response &res);
    //CHECK CONNECTION STATUS FUNCTION
    bool IsConnected()
    {
        return gkv_->GetSerialConnectionState();
    }
    //SEND RECEIVED DATA TO TOPICS FUNCTION
    void publishReceivedData(Gyrovert::GKV_PacketBase * buf);
};
#endif
