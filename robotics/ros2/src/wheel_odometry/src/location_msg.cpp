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

//#ifndef WHEEL_ODOMETRY_CAN_H_INCLUDED
//#define WHEEL_ODOMETRY_CAN_H_INCLUDED

using std::placeholders::_1;

//#include "wheel_odometry/location_msg.hpp"
class LocationMsg : public rclcpp::Node
{

public:
    /*!
        Constructor
    */
    LocationMsg()
    : Node("location_msg")
    {    
        RCLCPP_DEBUG(this->get_logger(), "Location Contructor");

        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        
        // Publishers
        m_location_pub = this->create_publisher<usr_msgs::msg::LocationMsg>("/custom_gps", default_qos);

        // Subscribers
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", default_qos, std::bind(&LocationMsg::ImuCb, this, _1));
        m_geo_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/wifi_geo/fix", default_qos, std::bind(&LocationMsg::GeoCb, this, _1));
        m_fix_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/fix", default_qos, std::bind(&LocationMsg::FixCb, this, _1));
    }

    /*!
        Destructor
    */
    ~LocationMsg(){};

    // Variables
    double m_imu_roll = 0.0f;
    double m_imu_pitch = 0.0f;
    double m_imu_yaw = 0.0f;
    double m_fix_lat = 0.0;
    double m_fix_long = 0.0;
    double m_geo_lat = 0.0;
    double m_geo_long = 0.0;
    usr_msgs::msg::LocationMsg m_location_msg;

private:
    // Subscribers callbacks

    void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

        tf2::Matrix3x3 m(q);
        m.getRPY(m_imu_roll, m_imu_pitch, m_imu_yaw);
    }
    void GeoCb(const usr_msgs::msg::LocationMsg::SharedPtr msg)
    {
        m_geo_lat = msg->latitude;
        m_geo_long = msg->longitude;
    }
    void FixCb(const usr_msgs::msg::LocationMsg::SharedPtr msg)
    {
        m_fix_lat = msg->latitude;
        m_fix_long = msg->longitude;
    }
    //Publisher
    rclcpp::Publisher<usr_msgs::msg::LocationMsg>::SharedPtr m_location_pub;
    //Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;           /*!< @sa ImuCb() */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_geo_sub;           /*!< @sa GeoCb() */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_fix_sub;           /*!< @sa FixCb() */

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    while (rclcpp::ok())
    {   
        m_location_msg.roll = m_imu_roll;
        m_location_msg.pitch = m_imu_pitch;
        m_location_msg.yaw = m_imu_yaw;
        m_location_msg.latitude = m_fix_lat;
        m_location_msg.longitude = m_fix_long;
        m_location_pub->publish(m_location_msg)

        rclcpp::spin(std::make_shared<LocationMsg>());
        //r.sleep();
    }
    rclcpp::shutdown();
    return 0;
}