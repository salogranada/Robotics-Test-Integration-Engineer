/*! @package pid_controller
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#ifndef PID_CONTROLLER_H_INCLUDED
#define PID_CONTROLLER_H_INCLUDED

// Custom libraries
#include "utils/console.hpp"

class PIDController
{
public:
    /*!
        Constructor of PID Controller class
        @see SoftSpeedSpline
        @see SpeedController
    */
    PIDController();

    /*!
        Destructor of PID Controller class
        @see SoftSpeedSpline
        @see SpeedController
    */
    ~PIDController(){};

public:
    /*!
        A PID controller (Feedforward + Feedback) calculate the control
        command for the specified velocity.
        @param ref_vz = Angular reference velocity (Z-axis)
        @param cur_wz = Current Kiwibot angular velocity (Z-axis)
        @param dt = Delta time
        @return Angular velocity control command
    */
    float SteeringPID(float ref_vz, float cur_wz, double dt);

    /*!
        A PID controller (Feedforward + Feedback) calculate the control
        command for the specified velocity.
        @param ref_vx = Linear reference velocity (X-axis)
        @param cur_vx = Current Kiwibot linear velocity (X-axis)
        @param dt = Delta time
        @return Linear velocity control command
    */
    float ThrottlePID(float ref_vx, float cur_vx, double dt);

private:
    // Environment variables
    bool m_throttle_ctrl = getEnv("SPEED_CONTROLLER_THROTTLE_CONTROL", true);
    bool m_steering_ctrl = getEnv("SPEED_CONTROLLER_STEERING_CONTROL", true);
    // Throttle
    float m_kp_thr = getEnv("SPEED_CONTROLLER_KP_THROTTLE", 0.2f);
    float m_ki_thr = getEnv("SPEED_CONTROLLER_KI_THROTTLE", 0.2f);
    float m_kd_thr = getEnv("SPEED_CONTROLLER_KD_THROTTLE", 0.0f);
    float m_kff_thr = getEnv("SPEED_CONTROLLER_FF_THROTTLE", 1.0f);
    // Steering
    float m_kp_str = getEnv("SPEED_CONTROLLER_KP_STEERING", 0.5f);
    float m_ki_str = getEnv("SPEED_CONTROLLER_KI_STEERING", 1.0f);
    float m_kd_str = getEnv("SPEED_CONTROLLER_KD_STEERING", 0.0f);
    float m_kff_str = getEnv("SPEED_CONTROLLER_FF_STEERING", 1.0f);
    // Velocities
    float m_max_linear_spd = getEnv("SPEED_CONTROLLER_MAX_LIN_VEL", 1.28f);
    float m_max_angular_spd = getEnv("SPEED_CONTROLLER_MAX_ANG_VEL_STATIC", 0.55f);
    // Member attributes
    double m_prev_prop_error = 0.0;
    double m_wz_int_error = 0.0;
    double m_vx_int_error = 0.0;
    float m_wz_prop_ek1 = 0.0f;
    float m_vx_prop_ek1 = 0.0f;
    float m_prev_ref_vx = 0.0f;
};
#endif
