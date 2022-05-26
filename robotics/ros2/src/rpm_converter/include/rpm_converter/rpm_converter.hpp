/*! @package rpm_converter
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#ifndef RPM_CONVERTER_H_INCLUDED
#define RPM_CONVERTER_H_INCLUDED

// Custom libraries
#include "utils/console.hpp"

// Custom Messages
#include "usr_msgs/msg/motors_rpm.hpp"

// ROS2 Default
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <rclcpp/rclcpp.hpp>

// ROS2 Messages
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include <math.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

using std::placeholders::_1;

class RpmConverter : public rclcpp::Node
{

public:
    /*!
        RpmConverter Class constructor. Inherits from rclcpp::Node
        @param options: rclcpp::NodeOptions.
    */
    RpmConverter(rclcpp::NodeOptions const &options);
    /*!
        RpmConverter Class destructor
        @param void
    */
    ~RpmConverter(){};

    /*!
        Publishes the motors RPM control commands to ROS2.
        @param void.
        @return void.
    */
    void PublishMotorsControl();

private:
    // Structures
    struct speed_cmd
    {
        float linear_vx = 0.0f;
        float angular_wz = 0.0f;
    } speed_ctrl;

    // Publishers
    rclcpp::Publisher<usr_msgs::msg::MotorsRPM>::SharedPtr
        m_motors_rpm_out_pub; /*!< Publish at topic /rpm_converter/motors_rpm_out */

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_spd_ctrl_out_sub; /*!< @sa SpeedControlOutCb() */

    // Timers
    rclcpp::TimerBase::SharedPtr m_publish_data_tmr; /*!< @sa PublisherTmrCb() */

    // Utils functions
    /*!
        Converts wheels angular velocity into RPMs.
        @param rads: Wheels angular velocities in floats.
        @return float: Corresponding RPMs in the range [-255, 255].
    */
    float RadsToDigital(float rads);

    /*!
        Function to calculate the right Angular Velocity
        @param lin_vx: const float& Linear velocity
        @param lin_wz: const float& Angular velocity
        @return std::vector<float>
    */
    std::vector<float> EnsureAngularVelocity(const float &lin_vx, const float &ang_wz);

    // Subscribers callbacks
    /*!
        Speed controller output subscription callback for the topic
       /motion_control/speed_controller/output_cmd.
        @param msg: Twist message.
        @return void.
    */
    void SpeedControlOutCb(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Timer CallBacks
    /*!
        Timer Cb to publish data in case
        @param void.
        @return void.
    */
    void PublisherTmrCb();

    /* Vectors for checking the motors state */
    std::vector<float> m_motors_rpm{0.0f, 0.0f, 0.0f, 0.0f};

    /* ROS messages for the robot state */
    usr_msgs::msg::MotorsRPM m_motors_rpm_feedback_msg;

    // Environment variables
    double m_robot_track = getEnv("ODOMETRY_CHASSIS_TRACK", 0.392f);
    double m_wheel_rad = getEnv("ODOMETRY_WHEEL_RADIUS", 0.079f);
    double m_wheel_max_rpm = getEnv("CONVERTER_WHEEL_MAX_RPM", 165.0f);
    double m_wheel_spd_factor = getEnv("CONVERTER_WHEEL_SPD_FACTOR", 17.6f);
    int m_publish_time = getEnv("CHASSIS_PUBLISH_TIME", 150);
};
#endif /* End of RPM_CONVERTER_H_INCLUDED */