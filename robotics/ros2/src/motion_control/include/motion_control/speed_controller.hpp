/*! @package speed_controller
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
        Develop by: Camilo Andres Alvis, Juan David Galvis and Davidson Rojas
*/

#ifndef SPEED_CONTROLLER_H_INCLUDED
#define SPEED_CONTROLLER_H_INCLUDED

// Custom objects
#include "motion_control/pid_controller.hpp"
#include "motion_control/soft_speed_spline.hpp"

// Custom libraries
#include "utils/console.hpp"

// ROS2 Default
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

// ROS2 Messages
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

// TF2 Transformations
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

// ROS2 Services
#include "std_srvs/srv/set_bool.hpp"

// STD Libraries
#include <math.h>
#include <memory>
#include <utility>
#include <vector>

#define TRANSITION_CONFIGURE 1
#define TRANSITION_CLEANUP 2
#define TRANSITION_ACTIVATE 3
#define TRANSITION_DEACTIVATE 4

using std::placeholders::_1;

class SpeedController : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
   public:
    /*!
        SpeedController Class constructor. Inherits from rclcpp_cascade_lifecycle::CascadeLifecycleNode
        @param options: rclcpp::NodeOptions.
    */
    explicit SpeedController(rclcpp::NodeOptions& options) : CascadeLifecycleNode("speed_controller", options) {}

    /*!
        SpeedController Class destructor
    */
    ~SpeedController(){};

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State&);

    /*!
        Main function in charge of performing velocity control (Linear and angular).
    */
    void Controller();

   private:
    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
        m_output_cmd_pub; /*!< Publish at topic /motion_control/speed_controller/output_cmd or /local_client/sim_control
                           */
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        m_ctrl_error_pub; /*!< Publish at topic /motion_control/speed_controller/error */

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_ctrl_cmd_sub; /*!< @sa ReferenceCommandsCb() */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odometry_sub;          /*!< @sa OdometryCb() */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;                 /*!< @sa Lambda Function */
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_sidis_state_sub;         /*!< @sa Lambda Function */
    // Subscribers callbacks
    /*!
        Reference speed controller commands subscription callback for the
        topic /motion_control/speed_controller/reference_cmd.
        @param msg: Twist Stamped message.
        @return void.
    */
    void ReferenceCommandsCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    /*!
        Global wheel odometry subscription callback for the topic /wheel_odometry/local_odometry.
        @param msg: Odometry message.
        @return void.
    */
    void OdometryCb(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Environment variables
    bool m_simulation = getEnv("LOCAL_SIMULATOR", false);
    float m_max_angular_velocity = getEnv("SPEED_CONTROLLER_MAX_ANG_VEL_STATIC", 0.55f);
    float m_avg_lin_acc = getEnv("SPEED_CONTROLLER_LINEAR_ACC_FACTOR", 2.0f);
    float m_avg_ang_acc = getEnv("SPEED_CONTROLLER_ANGULAR_ACC_FACTOR", 1.2f);
    float m_max_yaw_ang = getEnv("SPEED_CONTROLLER_MAX_YAW_ANG", 0.3f);

    // Objects
    std::shared_ptr<SoftSpeedSpline> m_linear_soft_spline;
    std::shared_ptr<SoftSpeedSpline> m_angular_soft_spline;
    std::shared_ptr<PIDController> m_pid_controller;

    // Member attributes
    rclcpp::Time m_prev_time;
    geometry_msgs::msg::TwistStamped m_robot_twist;
    geometry_msgs::msg::TwistStamped m_reference_cmd;
    bool m_yaw_ctrl = false;
    bool m_max_yaw_ctrl = false;
    double m_yaw = 0.0;
    double m_ref_yaw = 0.0;

   public:
    bool m_enable_controller = true;
    uint8_t m_state = TRANSITION_ACTIVATE;
};
#endif
