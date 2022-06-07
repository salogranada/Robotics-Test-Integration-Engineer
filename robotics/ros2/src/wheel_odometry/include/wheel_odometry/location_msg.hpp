// STD libraries
#include <math.h>
#include <memory>
#include <utility>
#include <vector>
// ROS2 Default
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
// TF2 Transformations
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// ROS2 Messages
#include "sensor_msgs/msg/imu.hpp"
// Custom Messages
#include "usr_msgs/msg/location_msg.hpp"
#include "usr_msgs/msg/motors_rpm.hpp"

#ifndef WHEEL_ODOMETRY_CAN_H_INCLUDED
#define WHEEL_ODOMETRY_CAN_H_INCLUDED

using std::placeholders::_1;

class LocationMsg : public rclcpp::Node
{
public:
    /*!
        Constructor
    */
    LocationMsg(rclcpp::NodeOptions &options);

    /*!
        Destructor
    */
    ~LocationMsg(){};


    void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg);


private:
    //Publisher
    rclcpp::Publisher<usr_msgs::msg::LocationMsg>::SharedPtr location_pub;
    //Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;           /*!< @sa ImuCb() */
    rclcpp::Subscription<usr_msgs::msg::LocationMsg>::SharedPtr m_geo_sub;           /*!< @sa GeoCb() */
    rclcpp::Subscription<usr_msgs::msg::LocationMsg>::SharedPtr m_fix_sub;           /*!< @sa FixCb() */

    //std::shared_ptr<tf2_ros::TransformBroadcaster> m_broadcaster;

    // Subscribers callbacks

    void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg);
    void GeoCb(const usr_msgs::msg::LocationMsg::SharedPtr msg);
    void FixCb(const usr_msgs::msg::LocationMsg::SharedPtr msg);


    // Variables
    double m_imu_roll = 0.0f;
    double m_imu_pitch = 0.0f;
    double m_imu_yaw = 0.0f;
    usr_msgs::msg::LocationMsg m_location_msg;

};

#endif