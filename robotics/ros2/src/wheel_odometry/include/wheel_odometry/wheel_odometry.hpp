/*! @package wheel_odometry
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
        Develop by: Camilo Andres Alvis and Davidson Rojas
*/
#ifndef WHEEL_ODOMETRY_CAN_H_INCLUDED
#define WHEEL_ODOMETRY_CAN_H_INCLUDED

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

// Transform broadcasting
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

// Custom libraries
#include "utils/console.hpp"

// ROS2 Messages
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

// ROS2 Services
#include "std_srvs/srv/set_bool.hpp"

// Custom Messages
#include "usr_msgs/msg/motors_rpm.hpp"

#define TRANSITION_CONFIGURE 1
#define TRANSITION_CLEANUP 2
#define TRANSITION_ACTIVATE 3
#define TRANSITION_DEACTIVATE 4

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class WheelOdometry : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
    /*!
        Constructor
        @param options: rclcpp::NodeOptions.
    */
    explicit WheelOdometry(rclcpp::NodeOptions &options) : CascadeLifecycleNode("wheel_odometry", options) {}

    /*!
        Destructor
    */
    ~WheelOdometry(){};

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &);

    /*!
        Function in charge of calculating robot movement base on wheel odometry
        @param void
        @return void
    */
    void CalculateOdometry();

    uint8_t m_state = TRANSITION_ACTIVATE;

private:
    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr
        m_wheel_odom_pub; /*!< Publish at topic /wheel_odometry/local_odometry */
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr
        m_wheel_odom_global_pub; /*!< Publish at topic /wheel_odometry/global_odometry */
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr
        m_absolute_distance_pub; /*!< Publish at topic /wheel_odometry/total_distance */

    // Subscribers
    rclcpp::Subscription<usr_msgs::msg::MotorsRPM>::SharedPtr m_motors_rpm_sub; /*!< @sa MotorsRPMCb() */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_imu_move_sub;        /*!< @sa MovementCb() */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;           /*!< @sa ImuCb() */

    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
        restart_srv_; /*!< Restart local odometry @sa RestartOdometryCb()*/

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_broadcaster;

    // Subscribers callbacks

    /*!
        Motors RPM subscription callback for the topic /uavcan/chassis/motors_rpm_feedback
        @param msg: Custom MotorsRPM message.
        @return void.
    */
    void MotorsRPMCb(const usr_msgs::msg::MotorsRPM::SharedPtr msg);

    /*!
        IMU information subscription callback for the topic /imu/data
        @param msg: IMU message
        @return void
    */
    void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg);

    /*!
        Restart local wheel odometry service callback.
        @param request_header: NaN.
        @param request: Contains the data.
        @param response: Contains the service response.
        @return bool: containing the service response.
    */
    bool RestartOdometryCb(const std::shared_ptr<rmw_request_id_t> request_header,
                           const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // Methods
    /*!
        Function in charge of calculating an approximate value for the slip
        by combining IMU angular velocity and direct kinematics
        @param kin_omega: Angular velocity found with direct kinematics.
        @param imu_omega: IMU angular velocity.
        @return float: Slip factor.
    */
    float CalculateSlipFactor(float kin_omega, float imu_omega);

    // Member attributes
    usr_msgs::msg::MotorsRPM m_motors_rpm;
    nav_msgs::msg::Odometry m_local_wheel_odom_msg;
    nav_msgs::msg::Odometry m_global_wheel_odom_msg;

    rclcpp::Time m_prev_time;

    // Environment variables
    double m_wheel_rad = getEnv("ODOMETRY_WHEEL_RADIUS", 0.075f);
    double m_chassis_track = getEnv("ODOMETRY_CHASSIS_TRACK", 0.37f);

    // Variables
    double m_imu_roll_offset = 0;
    double m_imu_pitch_offset = 0;
    double m_imu_yaw_offset = 0;
    double m_imu_roll_startup_offset = 0;  // Offset only for the first IMU reading
    double m_imu_pitch_startup_offset = 0; // Offset only for the first IMU reading
    double m_imu_yaw_startup_offset = 0;   // Offset only for the first IMU reading
    double m_imu_roll = 0.0f;
    double m_imu_pitch = 0.0f;
    double m_imu_yaw = 0.0f;
    double m_imu_omega = 0.0f;
    bool m_imu_published = false;
    bool m_imu_state = false;

    // Calculate total distance
    float m_previous_x = 0.0f;
    float m_previous_y = 0.0f;
    std_msgs::msg::Float32 m_total_distance;
};

#endif