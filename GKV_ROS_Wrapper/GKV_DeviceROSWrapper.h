#ifndef GKV_DEVICE_ROS_WRAPPER_H
#define GKV_DEVICE_ROS_WRAPPER_H
#include "ros/ros.h"
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
    uint8_t GKV_Status=0;
    uint8_t request_limit=100;
    bool ResetRequestFlag=false;
    bool SetAlgRequestFlag=false;
    bool CustomParamNumbersRequestFlag=false;
    bool CustomParamNumbersReceived=false;
    Gyrovert::GKV_CustomDataParam device_custom_parameters;

public:
    GKV_DeviceROSWrapper(ros::NodeHandle *nh, std::string serial_port, uint32_t baudrate)
    {
        //std::cout << serial_port <<std::endl;

        gkv_= new Gyrovert::GKV_Device(serial_port, baudrate);
        if (!(gkv_->GetSerialConnectionState())) GKV_Status = 1;
        received_adc_data_publisher=nh->advertise<gkv_ros_driver::GkvAdcData>("gkv_adc_data", 10);
        received_sensor_data_publisher=nh->advertise<gkv_ros_driver::GkvSensorsData>("gkv_sensors_data", 10);
        received_gyrovert_data_publisher=nh->advertise<gkv_ros_driver::GkvGyrovertData>("gkv_gyrovert_data", 10);
        received_inclinometer_data_publisher=nh->advertise<gkv_ros_driver::GkvInclinometerData>("gkv_inclinometer_data", 10);
        received_bins_data_publisher=nh->advertise<gkv_ros_driver::GkvBINSData>("gkv_bins_data", 10);
        received_gnss_data_publisher=nh->advertise<gkv_ros_driver::GkvGpsData>("gkv_gnss_data", 10);
        received_ext_gnss_data_publisher=nh->advertise<gkv_ros_driver::GkvExtGpsData>("gkv_ext_gnss_data", 10);
        received_custom_data_publisher=nh->advertise<gkv_ros_driver::GkvCustomData>("gkv_custom_data", 10);

        ResetService = nh->advertiseService("gkv_reset_srv", &GKV_DeviceROSWrapper::ResetDevice,this);
        SetAlgorithmService = nh->advertiseService("gkv_set_alg_srv", &GKV_DeviceROSWrapper::SetAlgorithm,this);

        if (!(GKV_Status))
        {
            gkv_->SetReceivedPacketCallback(std::bind(&GKV_DeviceROSWrapper::publishReceivedData, this, std::placeholders::_1));
            gkv_->RunDevice();
            gkv_->RequestCustomPacketParams();
        }
    }
    //RESET DEVICE FUNCTION
    bool ResetDevice(gkv_ros_driver::GkvReset::Request  &req,
                     gkv_ros_driver::GkvReset::Response &res)
    {
        ResetRequestFlag=true;
        for (uint8_t i=0;i<request_limit;i++)
        {
            gkv_->ResetDevice();
//            ROS_INFO("Device Reset Req [%d]",i);
            if (ResetRequestFlag==false)
            {
                break;
            }
        }
        res.result=(!(ResetRequestFlag));
        return res.result;
    }
    //SET DEVICE ALGORITHM FUNCTION
    bool SetAlgorithm(gkv_ros_driver::GkvSetAlgorithm::Request  &req,
                     gkv_ros_driver::GkvSetAlgorithm::Response &res)
    {
        if ((req.algorithm_number>9)||(req.algorithm_number==3))
        {
            return false;
        }
        SetAlgRequestFlag=true;
        for (uint8_t i=0;i<request_limit;i++)
        {
            gkv_->SetAlgorithm(req.algorithm_number);
//            ROS_INFO("Device Set Alg Req [%d]",i);
            if (SetAlgRequestFlag==false)
            {
                break;
            }
        }
        res.result=(!(SetAlgRequestFlag));
        return res.result;
    }
    //CHECK CONNECTION STATUS FUNCTION
    bool IsConnected()
    {
        return gkv_->GetSerialConnectionState();
    }
    //SEND RECEIVED DATA TO TOPICS FUNCTION
    void publishReceivedData(Gyrovert::GKV_PacketBase * buf)
    {
        char str[30];
        switch (buf->type)
        {
            case GKV_ADC_CODES_PACKET:
            {
                Gyrovert::GKV_ADCData* packet;
                packet = (Gyrovert::GKV_ADCData*)&buf->data;
                gkv_ros_driver::GkvAdcData msg;
                msg.acceleration_adc_x = packet->a[0];
                msg.acceleration_adc_y = packet->a[1];
                msg.acceleration_adc_z = packet->a[2];
                msg.angular_rate_adc_x = packet->w[0];
                msg.angular_rate_adc_y = packet->w[1];
                msg.angular_rate_adc_z = packet->w[2];
                received_adc_data_publisher.publish(msg);
                break;
            }
            case GKV_RAW_DATA_PACKET:
            {
                Gyrovert::GKV_RawData* packet;
                packet = (Gyrovert::GKV_RawData*)&buf->data;
                gkv_ros_driver::GkvSensorsData msg;
                msg.acceleration_x = packet->a[0];
                msg.acceleration_y = packet->a[1];
                msg.acceleration_z = packet->a[2];
                msg.angular_rate_x = packet->w[0];
                msg.angular_rate_y = packet->w[1];
                msg.angular_rate_z = packet->w[2];
                received_sensor_data_publisher.publish(msg);
                break;
            }
            case GKV_EULER_ANGLES_PACKET:
            {
                Gyrovert::GKV_GyrovertData* packet;
                packet = (Gyrovert::GKV_GyrovertData*)&buf->data;
                gkv_ros_driver::GkvGyrovertData msg;
                msg.yaw = packet->yaw;
                msg.pitch = packet->pitch;
                msg.roll = packet->roll;
                received_gyrovert_data_publisher.publish(msg);
                break;
            }
            case GKV_INCLINOMETER_PACKET:
            {
                Gyrovert::GKV_InclinometerData* packet;
                packet = (Gyrovert::GKV_InclinometerData*)&buf->data;
                gkv_ros_driver::GkvInclinometerData msg;
                msg.alpha = packet->alfa;
                msg.beta = packet->beta;
                received_inclinometer_data_publisher.publish(msg);
                break;
            }
            case GKV_BINS_PACKET:
            {
                Gyrovert::GKV_BINSData* packet;
                packet = (Gyrovert::GKV_BINSData*)&buf->data;
                gkv_ros_driver::GkvBINSData msg;
                msg.x = packet->x;
                msg.y = packet->y;
                msg.z = packet->z;
                msg.yaw = packet->yaw;
                msg.pitch = packet->pitch;
                msg.roll = packet->roll;
                msg.alpha = packet->alfa;
                msg.beta = packet->beta;
                msg.q3 = packet->q[0];
                msg.q2 = packet->q[1];
                msg.q1 = packet->q[2];
                msg.q0 = packet->q[3];
                received_bins_data_publisher.publish(msg);
                break;
            }
            case GKV_GNSS_PACKET:
            {
                Gyrovert::GKV_GpsData* packet;
                packet = (Gyrovert::GKV_GpsData*)&buf->data;
                gkv_ros_driver::GkvGpsData msg;
                msg.time = packet->time;
                msg.latitude = packet->latitude;
                msg.longitude = packet->longitude;
                msg.altitude = packet->altitude;
                msg.gps_state_status = packet->state_status;
                msg.TDOP = packet->TDOP;
                msg.HDOP = packet->HDOP;
                msg.VDOP = packet->VDOP;
                msg.horizontal_vel = packet->velocity;
                msg.azimuth = packet->yaw;
                msg.vertical_vel = packet->alt_velocity;
                received_gnss_data_publisher.publish(msg);
                break;
            }
            case GKV_EXTENDED_GNSS_PACKET:
            {
                Gyrovert::GKV_GpsDataExt* packet;
                packet = (Gyrovert::GKV_GpsDataExt*)&buf->data;
                gkv_ros_driver::GkvExtGpsData msg;
                msg.lat_vel = packet->vlat;
                msg.lon_vel = packet->vlon;
                msg.lat_std = packet->sig_lat;
                msg.lon_std = packet->sig_lon;
                msg.alt_std = packet->sig_alt;
                msg.lat_vel_std = packet->sig_vlat;
                msg.lon_vel_std = packet->sig_vlon;
                msg.alt_vel_std = packet->sig_valt;
                msg.num_ss = packet->num_ss;
                ROS_INFO("Extended GNSS Data: lat_vel=[%f] lon_vel=[%f] lat_std=[%f] lon_std=[%f] alt_std=[%f] lat_vel_std=[%f] lon_vel_std=[%f] alt_vel_std=[%f] num_ss=[%d]",
                                packet->vlat, packet->vlon, packet->sig_lat, packet->sig_lon, packet->sig_alt, packet->sig_vlat, packet->sig_vlon, packet->sig_valt, packet->num_ss);
                received_ext_gnss_data_publisher.publish(msg);
                break;
            }
            case GKV_CUSTOM_PACKET:
            {
                Gyrovert::GKV_CustomData* packet;
                packet = (Gyrovert::GKV_CustomData*)&buf->data;
                gkv_ros_driver::GkvCustomData msg;
                if (CustomParamNumbersReceived)
                {
                    msg.quantity_of_params=device_custom_parameters.num;
                    //select parameters that have uint32_t structure
                    for (uint8_t i=0;i<device_custom_parameters.num;i++)
                    {
                        msg.numbers.push_back(device_custom_parameters.param[i]);
                        if ((device_custom_parameters.param[i]==GKV_ALG_INT_LAT_NOPH)||
                            (device_custom_parameters.param[i]==GKV_ALG_INT_LON_NOPH)||
                            (device_custom_parameters.param[i]==GKV_ALG_INT_ALT_NOPH)||
                            (device_custom_parameters.param[i]==GKV_UTC_TIME)||
                            (device_custom_parameters.param[i]==GKV_GNSS_STATUS)||
                            (device_custom_parameters.param[i]==GKV_ALG_INT_LAT)||
                            (device_custom_parameters.param[i]==GKV_ALG_INT_LON)||
                            (device_custom_parameters.param[i]==GKV_GNSS_INT_LAT)||
                            (device_custom_parameters.param[i]==GKV_GNSS_INT_LON)||
                            (device_custom_parameters.param[i]==GKV_ALG_STATE_STATUS)||
                            (device_custom_parameters.param[i]==GKV_GPS_INT_X)||
                            (device_custom_parameters.param[i]==GKV_GPS_INT_Y)||
                            (device_custom_parameters.param[i]==GKV_GPS_INT_Z))
                        {
                            msg.param_values.push_back(*((int32_t *)&packet->parameter[i]));
                        }
                        else {
                            msg.param_values.push_back(packet->parameter[i]);
                        }
                    }
                    received_custom_data_publisher.publish(msg);
                }
                else
                {
                    gkv_->RequestCustomPacketParams();
                }
                break;
            }
            case GKV_CUSTOM_DATA_PARAM_PACKET:
            {
                Gyrovert::GKV_CustomDataParam* packet;
                packet = (Gyrovert::GKV_CustomDataParam*)&buf->data;
                memcpy(&device_custom_parameters,packet,sizeof(Gyrovert::GKV_CustomDataParam));
                CustomParamNumbersReceived=true;
                break;
            }
            case GKV_CONFIRM_PACKET:
            {
                if (ResetRequestFlag)
                {
                    ResetRequestFlag=false;
//                    ROS_INFO("Device Reseted");
                }
                if (SetAlgRequestFlag)
                {
                    SetAlgRequestFlag=false;
//                    ROS_INFO("Algorithm changed");
                }
                break;
            }
        }
    }
};
#endif
