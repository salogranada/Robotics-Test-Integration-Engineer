/*! @package speed_controller
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
        Develop by: Camilo Andres Alvis, Juan David Galvis and Davidson Rojas
*/

#include "motion_control/speed_controller.hpp"

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SpeedController::on_configure(
    const rclcpp_lifecycle::State &)
{
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    m_output_cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/motion_control/speed_controller/output_cmd", default_qos);

    m_ctrl_error_pub =
        this->create_publisher<geometry_msgs::msg::TwistStamped>("/motion_control/speed_controller/error", default_qos);

    // Subscribers
    m_ctrl_cmd_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/motion_control/speed_controller/reference_cmd", default_qos,
        std::bind(&SpeedController::ReferenceCommandsCb, this, _1));

    m_odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odometry/local_odometry", default_qos, std::bind(&SpeedController::OdometryCb, this, _1));

    /* Time to detect the dt inside the control loop */
    m_prev_time = this->now();

    /* Soft Speed object */
    m_linear_soft_spline = std::make_shared<SoftSpeedSpline>(m_avg_lin_acc);
    m_angular_soft_spline = std::make_shared<SoftSpeedSpline>(m_avg_ang_acc);

    /* PID Controller object */
    m_pid_controller = std::make_shared<PIDController>();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SpeedController::on_activate(
    const rclcpp_lifecycle::State &)
{
    m_output_cmd_pub->on_activate();
    m_ctrl_error_pub->on_activate();
    m_state = TRANSITION_ACTIVATE;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SpeedController::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    m_state = TRANSITION_DEACTIVATE;
    m_output_cmd_pub->on_deactivate();
    m_ctrl_error_pub->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SpeedController::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    m_state = TRANSITION_CLEANUP;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SpeedController::on_shutdown(
    const rclcpp_lifecycle::State &)
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void SpeedController::OdometryCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    m_robot_twist.twist = msg->twist.twist;
}

void SpeedController::ReferenceCommandsCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    m_reference_cmd.header.stamp = this->now();
    m_reference_cmd.twist = msg->twist;
}

void SpeedController::Controller()
{
    /* Reference velocities */
    float lin_vx = m_reference_cmd.twist.linear.x;
    float ang_wz = m_reference_cmd.twist.angular.z;

    rclcpp::Time curr_time = this->now();
    double dt = (curr_time - m_prev_time).seconds();
    m_prev_time = this->now();

    float vx_ref = m_linear_soft_spline->CalculateSoftSpeed(lin_vx, true);

    ang_wz = lin_vx == 0.0f ? m_angular_soft_spline->CalculateSoftSpeed(ang_wz, false) : ang_wz;

    /* Angular velocity control or yaw control */
    float ctrl_ang_vel = m_pid_controller->SteeringPID(ang_wz, m_robot_twist.twist.angular.z, dt);
    /* Linear velocity control */
    float ctrl_lin_vel = m_pid_controller->ThrottlePID(vx_ref, m_robot_twist.twist.linear.x, dt);

    auto output_cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
    output_cmd_msg->linear.x = ctrl_lin_vel;
    output_cmd_msg->angular.z = ctrl_ang_vel;
    m_output_cmd_pub->publish(std::move(output_cmd_msg));

    /********************************************
     * Fill, Calculate, and Publish the error message
     *
     ********************************************/
    // Use this message
    auto output_error_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    // How could I publish a message of type std::shared_ptr?
    /********************************************
     * END CODE
     *  ********************************************/
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto speed_node = std::make_shared<SpeedController>(options);

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(speed_node->get_node_base_interface());

    RCLCPP_WARN(speed_node->get_logger(), "Init Speed Controller");
    /* Node set to 30 Hz */
    auto period = std::chrono::milliseconds(33);
    rclcpp::Rate r(period);

    speed_node->trigger_transition(TRANSITION_CONFIGURE);
    speed_node->trigger_transition(TRANSITION_ACTIVATE);
    while (rclcpp::ok())
    {
        executor->spin_some();
        if (speed_node->m_enable_controller && speed_node->m_state == TRANSITION_ACTIVATE)
        {
            speed_node->Controller();
        }
        r.sleep();
    }
    rclcpp::shutdown();

    return 0;
}