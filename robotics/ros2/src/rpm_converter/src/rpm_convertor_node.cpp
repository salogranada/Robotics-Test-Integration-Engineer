/*! @package rpm_converter
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "rpm_converter/rpm_converter.hpp"

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RpmConverter::on_configure(
    const rclcpp_lifecycle::State &)
{
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

  // Publishers
  m_motors_rpm_out_pub =
      this->create_publisher<usr_msgs::msg::MotorsRPM>("/rpm_converter/motors_rpm_out", default_qos);

  // Subscribers
  m_spd_ctrl_out_sub =
      this->create_subscription<geometry_msgs::msg::Twist>("/motion_control/speed_controller/output_cmd", default_qos,
                                                           std::bind(&RpmConverter::SpeedControlOutCb, this, _1));

  // Timers
  m_publish_data_tmr = this->create_wall_timer(std::chrono::milliseconds(m_publish_time),
                                               std::bind(&RpmConverter::PublisherTmrCb, this));

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RpmConverter::on_activate(
    const rclcpp_lifecycle::State &)
{
    m_motors_rpm_out_pub->on_activate();
    m_state = TRANSITION_ACTIVATE;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RpmConverter::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    m_state = TRANSITION_DEACTIVATE;
    m_motors_rpm_out_pub->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RpmConverter::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    m_state = TRANSITION_CLEANUP;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RpmConverter::on_shutdown(
    const rclcpp_lifecycle::State &)
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void RpmConverter::SpeedControlOutCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  m_publish_data_tmr->cancel();

  speed_ctrl.linear_vx = msg->linear.x;
  speed_ctrl.angular_wz = msg->angular.z;

  m_publish_data_tmr->reset();
}

// Timers Callbacks
void RpmConverter::PublisherTmrCb()
{
  m_publish_data_tmr->cancel();
  speed_ctrl.linear_vx = 0.0f;
  speed_ctrl.angular_wz = 0.0f;
}

// Member methods
std::vector<float> RpmConverter::EnsureAngularVelocity(const float &lin_vx, const float &ang_wz)
{
  /**
   * Velocity range[17.6, -17.6]
   * */
  float out_lin_vx = 0.0f;
  float out_ang_wz = 0.0f;

  float vel_r = 0.0f; /* Right wheels velocity */
  float vel_l = 0.0f; /* Left wheels velocity */

  /* This depends on the maximum motors RPMS */
  float max_lin_vx = m_wheel_spd_factor;  /* Factor for max velocity including the radius  */
  float min_lin_vx = -m_wheel_spd_factor; /* Factor for min velocity including the radius */

  float abs_lin_vx = std::fabs(lin_vx);
  float abs_ang_wz = std::fabs(ang_wz);

  if (abs_lin_vx > 0.0f)
  {
    /**
     * Converting velocities to differential - Direct Kinematics
     * */
    float v_r_diff = (2.0f * lin_vx + ang_wz * m_robot_track) / (2.0f * m_wheel_rad);
    float v_l_diff = (2.0f * lin_vx - ang_wz * m_robot_track) / (2.0f * m_wheel_rad);

    /**
     * Find maximum velocity between the two
     * */
    float vel_rl_max = std::max(v_r_diff, v_l_diff);
    float vel_rl_min = std::min(v_r_diff, v_l_diff);

    if (vel_rl_max > max_lin_vx)
    {
      vel_r = v_r_diff - (vel_rl_max - max_lin_vx); /* Output right velocity */
      vel_l = v_l_diff - (vel_rl_max - max_lin_vx); /* Output left velocity */
    }

    else if (vel_rl_min < min_lin_vx)
    {
      vel_r = v_r_diff + (min_lin_vx - vel_rl_min); /* Output right velocity */
      vel_l = v_l_diff + (min_lin_vx - vel_rl_min); /* Output left velocity */
    }

    else
    {
      vel_r = v_r_diff;
      vel_l = v_l_diff;
    }

    /* Inverse Kinematics */
    out_lin_vx = (m_wheel_rad / 2.0f) * (vel_r + vel_l);
    out_ang_wz = (m_wheel_rad / m_robot_track) * (vel_r - vel_l);

    out_lin_vx = std::copysign(out_lin_vx, lin_vx);
    out_ang_wz = std::copysign(out_ang_wz, ang_wz);
  }

  else
  {
    /**
     * The robot is not moving
     * We are not able to move or rotate to a minimum angular speed
     * */
    float max_ang_wz = (m_wheel_rad / m_robot_track) * (2 * max_lin_vx);
    float min_ang_wz = (m_wheel_rad / m_robot_track) * (2 * min_lin_vx);

    if (abs_ang_wz > min_ang_wz)
    {
      out_ang_wz = std::max(std::min(abs_ang_wz, max_ang_wz), min_ang_wz);
      out_ang_wz = std::copysign(out_ang_wz, ang_wz);
    }

    else
    {
      out_lin_vx = 0.0f;
      out_ang_wz = 0.0f;
    }
  }

  float v_r_out = (2.0f * out_lin_vx + out_ang_wz * m_robot_track) / (2.0f * m_wheel_rad);
  float v_l_out = (2.0f * out_lin_vx - out_ang_wz * m_robot_track) / (2.0f * m_wheel_rad);

  std::vector<float> out_velocities{v_r_out, v_l_out};
  return out_velocities;
}

float RpmConverter::RadsToDigital(float rads)
{
  /**
   * Normalization of the RPMs to get the digital value
   * Wheel max RPM depends on the motors
   * */
  float rpm = rads * 60.0f / (2 * M_PI);
  rpm = rpm > m_wheel_max_rpm ? m_wheel_max_rpm : rpm;
  rpm = rpm < -m_wheel_max_rpm ? -m_wheel_max_rpm : rpm;
  return rpm;
}

void RpmConverter::PublishMotorsControl()
{
  m_publish_data_tmr->cancel();

  /* Perform a comparision at this point */
  std::vector<float> ctrl_velocities = EnsureAngularVelocity(speed_ctrl.linear_vx, speed_ctrl.angular_wz);

  float w_right = ctrl_velocities[0];
  float w_left = ctrl_velocities[1];

  /* Right is negative to match the sense of motion */
  float w_right_dig = -RadsToDigital(w_right);
  float w_left_dig = RadsToDigital(w_left);

  /* Motors RPMs after being processed */
  std::vector<float> motors_rpm{w_right_dig, w_right_dig, w_left_dig, w_left_dig};

  auto out_rpms_msg = usr_msgs::msg::MotorsRPM();
  out_rpms_msg.rpms_fr = motors_rpm[0];
  out_rpms_msg.rpms_rr = motors_rpm[1];
  out_rpms_msg.rpms_rl = motors_rpm[2];
  out_rpms_msg.rpms_fl = motors_rpm[3];
  m_motors_rpm_out_pub->publish(out_rpms_msg);

  m_publish_data_tmr->reset();
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  
  auto rpm_converter_node = std::make_shared<RpmConverter>(options);
  auto executor =std::make_shared<rclcpp::executors::SingleThreadedExecutor>();;
  executor.add_node(rpm_converter_node);

  auto period = std::chrono::milliseconds(rpm_converter_node->m_publish_time);
  rclcpp::Rate r(period);

  rpm_converter_node->trigger_transition(TRANSITION_CONFIGURE);
  rpm_converter_node->trigger_transition(TRANSITION_ACTIVATE);
  while (rclcpp::ok())
  {
    executor.spin_some();
    if (rpm_converter_node->m_state == TRANSITION_ACTIVATE)
        {
          rpm_converter_node->PublishMotorsControl();
        }
    r.sleep();
  }
  rclcpp::shutdown();

  return 0;
}