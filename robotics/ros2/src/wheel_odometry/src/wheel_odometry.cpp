/*! @package wheel_odometry
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
        Develop by: Camilo Andres Alvis and Davidson Rojas
*/

#include "wheel_odometry/wheel_odometry.hpp"
#define PI 3.1516
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn WheelOdometry::on_configure(
    const rclcpp_lifecycle::State &)
{
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    // Publishers
    m_wheel_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odometry/local_odometry", default_qos);

    m_wheel_odom_global_pub =
        this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odometry/global_odometry", default_qos);

    m_absolute_distance_pub =
        this->create_publisher<std_msgs::msg::Float32>("/wheel_odometry/total_distance", default_qos);

    // Subscribers
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", default_qos,
                                                                 std::bind(&WheelOdometry::ImuCb, this, _1));

    m_motors_rpm_sub = this->create_subscription<usr_msgs::msg::MotorsRPM>(
        "/uavcan/chassis/motors_rpm_feedback", default_qos, std::bind(&WheelOdometry::MotorsRPMCb, this, _1));

    // Services
    restart_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/wheel_odometry/restart", std::bind(&WheelOdometry::RestartOdometryCb, this, _1, _2, _3));

    m_local_wheel_odom_msg.header.frame_id = "local_odom";
    m_local_wheel_odom_msg.child_frame_id = "base_link";

    m_global_wheel_odom_msg.header.frame_id = "odom";
    m_global_wheel_odom_msg.child_frame_id = "base_link";

    // Broadcast transform pointer
    m_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_prev_time = this->now();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn WheelOdometry::on_activate(
    const rclcpp_lifecycle::State &)
{
    m_wheel_odom_pub->on_activate();
    m_wheel_odom_global_pub->on_activate();
    m_absolute_distance_pub->on_activate();
    m_state = TRANSITION_ACTIVATE;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn WheelOdometry::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    m_state = TRANSITION_DEACTIVATE;
    m_wheel_odom_pub->on_deactivate();
    m_wheel_odom_global_pub->on_deactivate();
    m_absolute_distance_pub->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn WheelOdometry::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    m_state = TRANSITION_CLEANUP;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn WheelOdometry::on_shutdown(
    const rclcpp_lifecycle::State &)
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Subscribers callbacks
bool WheelOdometry::RestartOdometryCb(const std::shared_ptr<rmw_request_id_t> request_header,
                                      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    (void)request_header;
    (void)request;
    /**
     * Restarts the local odometry in this way, our current pose becomes our zero
     * */

    RCLCPP_WARN(this->get_logger(), "Restarting odometry");

    m_local_wheel_odom_msg.pose.pose.position.x = 0.0f;
    m_local_wheel_odom_msg.pose.pose.position.y = 0.0f;
    m_local_wheel_odom_msg.pose.pose.position.z = 0.0f;

    m_imu_roll_offset = m_imu_roll;
    m_imu_pitch_offset = m_imu_pitch;
    m_imu_yaw_offset = m_imu_yaw;

    // Quaternion representation
    tf2::Quaternion quat;
    quat.setRPY(0.0f, 0.0f, 0.0f);

    // Quaternion assignation
    m_local_wheel_odom_msg.pose.pose.orientation.x = quat[0];
    m_local_wheel_odom_msg.pose.pose.orientation.y = quat[1];
    m_local_wheel_odom_msg.pose.pose.orientation.z = quat[2];
    m_local_wheel_odom_msg.pose.pose.orientation.w = quat[3];

    if (m_state == TRANSITION_ACTIVATE)
    {
        m_wheel_odom_pub->publish(m_local_wheel_odom_msg);
    }

    // Service response
    response->success = true;
    response->message = "Odometry Restarted";

    return true;
}

void WheelOdometry::MotorsRPMCb(const usr_msgs::msg::MotorsRPM::SharedPtr msg)
{
    m_motors_rpm.rpms_fr = msg->rpms_fr;
    m_motors_rpm.rpms_rr = msg->rpms_rr;
    m_motors_rpm.rpms_rl = msg->rpms_rl;
    m_motors_rpm.rpms_fl = msg->rpms_fl;
}

void WheelOdometry::ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(m_imu_roll, m_imu_pitch, m_imu_yaw);
    m_imu_omega = msg->angular_velocity.z;

    if (!m_imu_published) // is first callback
    {
        // IMU state must be OK
        m_imu_published = true;

        // in first callback we initialize the offsets
        m_imu_roll_startup_offset = m_imu_roll;
        m_imu_pitch_startup_offset = m_imu_pitch;
        m_imu_yaw_startup_offset = m_imu_yaw;

        m_imu_roll_offset = m_imu_roll_startup_offset;
        m_imu_pitch_offset = m_imu_pitch_startup_offset;
        m_imu_yaw_offset = m_imu_yaw_startup_offset;
    }
}

float WheelOdometry::CalculateSlipFactor(float kin_omega, float imu_omega)
{
    float alpha = 1.0f;
    if (kin_omega != 0.0f)
    {
        alpha = 1.0f - ((kin_omega - imu_omega) / 2.0f);
        alpha = alpha < 0.0f ? 0.0f : alpha;
        alpha = alpha > 1.0f ? 1.0f : alpha;
    }

    return alpha;
}

void WheelOdometry::CalculateOdometry()
{
    /**
     * Using kinematic calculation instead of IMU
     * While testing this will be used
     * */
    rclcpp::Time curr_time = this->now();

    /* Wheels linear velocities */
    /********************************************
     * Calculate your Amazing Linear Velocity for each Wheel HERE
    float FR_vel = ?;
    float RR_vel = ?;
    float RL_vel = ?;
    float FL_vel = ?;
    /********************************************
     * END CODE
     *  ********************************************/

    /* Left and Right linear velocities */
    float R_vel = (FR_vel + RR_vel) / 2.0f;
    float L_vel = (FL_vel + RL_vel) / 2.0f;

    /* Lineal and angular velocities */
    float X_vel = (R_vel + L_vel) / 2.0f;
    float kin_omega = (R_vel - L_vel) / m_chassis_track;

    double dt = (curr_time - m_prev_time).seconds();

    /* Filling a Quaternion with the current euler angles */
    tf2::Quaternion quat;
    quat.setRPY(m_imu_roll - m_imu_roll_offset, m_imu_pitch - m_imu_pitch_offset, m_imu_yaw - m_imu_yaw_offset);

    m_local_wheel_odom_msg.header.stamp = this->now();
    m_local_wheel_odom_msg.pose.pose.orientation.x = quat[0];
    m_local_wheel_odom_msg.pose.pose.orientation.y = quat[1];
    m_local_wheel_odom_msg.pose.pose.orientation.z = quat[2];
    m_local_wheel_odom_msg.pose.pose.orientation.w = quat[3];

    /* Speeds in X and Y axes */
    float X_dot = X_vel * cos(m_imu_yaw - m_imu_yaw_offset);
    float Y_dot = X_vel * sin(m_imu_yaw - m_imu_yaw_offset);

    /* Adding up the displacement */

    /* Wheels linear velocities */
    /********************************************
     * Calculate the X and Y positions
    float delta_X = ?;
    float delta_Y = ?;

    // Don't forget the offset :smile:
    float X = ?;
    float Y = ?;

    m_local_wheel_odom_msg.pose.pose.position.x = ?;
    m_local_wheel_odom_msg.pose.pose.position.y = ?;
    /********************************************
     * END CODE
     *  ********************************************/

    m_local_wheel_odom_msg.pose.pose.position.z = 0.0f;
    m_local_wheel_odom_msg.pose.covariance = {0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.2, 0, 0, 0,
                                              0, 0, 0, 0.001, 0, 0, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0, 0, 0.002};

    /* Velocities assignation */
    m_local_wheel_odom_msg.twist.twist.linear.x = X_vel;
    m_local_wheel_odom_msg.twist.twist.linear.y = 0.0f;
    m_local_wheel_odom_msg.twist.twist.linear.z = 0.0f;
    m_local_wheel_odom_msg.twist.twist.angular.x = 0.0f;
    m_local_wheel_odom_msg.twist.twist.angular.y = 0.0f;
    m_local_wheel_odom_msg.twist.twist.angular.z = kin_omega;

    /* Testing purposes */
    m_local_wheel_odom_msg.twist.covariance = {0.0000001, 0, 0, 0, 0, 0, 0, 0.000000001, 0, 0, 0, 0,
                                               0, 0, 0.00000002, 0, 0, 0, 0, 0, 0, 0.01, 0, 0,
                                               0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.002};

    // Calculations for Global Wheel Odometry
    float alpha = CalculateSlipFactor(kin_omega, m_imu_omega);
    float X_vel_corrected = X_vel * alpha;

    // Filling a global Quaternion with the current euler angles
    tf2::Quaternion quat_global;
    quat_global.setRPY(m_imu_roll - m_imu_roll_startup_offset, m_imu_pitch - m_imu_pitch_startup_offset,
                       m_imu_yaw - m_imu_yaw_startup_offset);

    m_global_wheel_odom_msg.header.stamp = this->now();
    m_global_wheel_odom_msg.pose.pose.orientation.x = quat_global[0];
    m_global_wheel_odom_msg.pose.pose.orientation.y = quat_global[1];
    m_global_wheel_odom_msg.pose.pose.orientation.z = quat_global[2];
    m_global_wheel_odom_msg.pose.pose.orientation.w = quat_global[3];

    // Speeds in X and Y axes
    float X_dot_global = X_vel_corrected * cos(m_imu_yaw - m_imu_yaw_startup_offset);
    float Y_dot_global = X_vel_corrected * sin(m_imu_yaw - m_imu_yaw_startup_offset);

    // Adding displacement in [m] to the global message

    /********************************************
     * Calculate the X and Y positions
    float delta_X_global = ?;
    float delta_Y_global = ?;

    // Don't forget the offset :smile:
    float X_global = ?;
    float Y_global = ?;

    m_global_wheel_odom_msg.pose.pose.position.x = ?;
    m_global_wheel_odom_msg.pose.pose.position.y = ?;
    /********************************************
     * END CODE
     *  ********************************************/

    m_global_wheel_odom_msg.pose.pose.position.z = 0.0f;
    m_global_wheel_odom_msg.pose.covariance = m_local_wheel_odom_msg.pose.covariance;

    // Velocities assignation
    m_global_wheel_odom_msg.twist.twist.linear.x = X_vel_corrected;
    m_global_wheel_odom_msg.twist.twist.linear.y = 0.0f;
    m_global_wheel_odom_msg.twist.twist.linear.z = 0.0f;
    m_global_wheel_odom_msg.twist.twist.angular.x = 0.0f;
    m_global_wheel_odom_msg.twist.twist.angular.y = 0.0f;
    m_global_wheel_odom_msg.twist.twist.angular.z = kin_omega;
    m_global_wheel_odom_msg.twist.covariance = m_local_wheel_odom_msg.twist.covariance;

    if (m_imu_published)
    {
        m_wheel_odom_pub->publish(m_local_wheel_odom_msg);
        m_wheel_odom_global_pub->publish(m_global_wheel_odom_msg);
        // Calculate total distance

        /********************************************
         * Your Amazing tachometer
        float d_increment = ?;

        // Update previous data
        m_previous_x = ?;
        m_previous_y = ?;

        // Accumulate distance
        m_total_distance.data = ?;
        /********************************************
         * END CODE
         *  ********************************************/

        // Publish total distance
        m_absolute_distance_pub->publish(m_total_distance);

        // Broadcast transform between odom and base_link
        geometry_msgs::msg::TransformStamped broadcaster_msg;
        broadcaster_msg.transform.translation.x = m_global_wheel_odom_msg.pose.pose.position.x;
        broadcaster_msg.transform.translation.y = m_global_wheel_odom_msg.pose.pose.position.y;
        broadcaster_msg.transform.translation.z = m_global_wheel_odom_msg.pose.pose.position.z;
        broadcaster_msg.transform.rotation.x = m_global_wheel_odom_msg.pose.pose.orientation.x;
        broadcaster_msg.transform.rotation.y = m_global_wheel_odom_msg.pose.pose.orientation.y;
        broadcaster_msg.transform.rotation.z = m_global_wheel_odom_msg.pose.pose.orientation.z;
        broadcaster_msg.transform.rotation.w = m_global_wheel_odom_msg.pose.pose.orientation.w;
        broadcaster_msg.header.stamp = m_global_wheel_odom_msg.header.stamp;
        broadcaster_msg.header.frame_id = "odom";
        broadcaster_msg.child_frame_id = "base_link";

        // Publish to /tf
        m_broadcaster->sendTransform(broadcaster_msg);
    }

    m_prev_time = this->now();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto odom_node = std::make_shared<WheelOdometry>(options);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(odom_node->get_node_base_interface());

    /* Node set to 30 Hz */
    auto period = std::chrono::milliseconds(33);
    rclcpp::Rate r(period);

    odom_node->trigger_transition(TRANSITION_CONFIGURE);
    odom_node->trigger_transition(TRANSITION_ACTIVATE);
    while (rclcpp::ok())
    {
        executor->spin_some();
        if (odom_node->m_state == TRANSITION_ACTIVATE)
        {
            odom_node->CalculateOdometry();
        }
        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}