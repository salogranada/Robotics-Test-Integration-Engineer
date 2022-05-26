/*! @package soft_speed_spline
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#ifndef SOFT_SPEED_SPLINE_H_INCLUDED
#define SOFT_SPEED_SPLINE_H_INCLUDED

// Custom libraries
#include "utils/console.hpp"

// ROS2 Default
#include "rclcpp/rclcpp.hpp"

// STD Libraries
#include <cmath>

enum
{
    steady = 1,
    acceleration = 2
};

class SoftSpeedSpline
{
public:
    /*!
        SoftSpeedSpline Class constructor.
        @param avg_acc: Average acceleration value to calculate the velocity spline.
    */
    SoftSpeedSpline(float avg_acc);

    /*!
        SoftSpeedSpline Class destructor.
    */
    ~SoftSpeedSpline(){};

public:
    /*!
        Description:
        - The Soft speed profile proposed is divided intro three different stages
        - The overall profile shape is a second order polynomial
        @param reference_vel: Reference velocity (Given by the speed controller)
        @param type: type: 0 angular, 1 linear
        @return out_vel_: Reference velocity given by the speed profile
    */
    float CalculateSoftSpeed(float reference_vel, bool type);

private:
    /*!
        Description:
        - Coefficients calculation for the Soft speed profile
        @param init_v: Initial velocity
        @param final_v: Final velocity
        @param initial_acc: Initial acceleration
        @return Parameters to a global variable
    */
    void SplineCoefficients(float init_v, float final_v, float init_acc);

    /*!
        Description:
        - The Soft speed profile proposed is divided intro three different stages
        - The overall profile shape is a second order polynomial
        @param curr_time: Current time in the speed profile (Spline)
    */
    float SoftSpeedValue(double curr_time);

private:
    // Member attributes
    rclcpp::Clock m_clock;
    rclcpp::Time m_start_time;
    rclcpp::Time m_current_time;
    bool m_soft_speed = getEnv("SOFT_SPEED_ENABLE", true);
    float m_kiwibot_max_spd = getEnv("SPEED_CONTROLLER_MAX_LIN_VEL", 1.28f);
    bool m_manual_cmd = false;
    double m_time_1 = 0.0;
    double m_time_2 = 0.0;
    float spline_params[3][3];
    float m_avg_acc = 0.0;
    float m_out_vel = 0.0;
    float m_target_vel = 0.0;
    int m_curve_stage = steady;
};
#endif // Implemented inside the lib/libsoft_speed.a