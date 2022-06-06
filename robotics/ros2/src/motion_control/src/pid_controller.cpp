/*! @package pid_controller
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "motion_control/pid_controller.hpp"

PIDController::PIDController() {}

float PIDController::ThrottlePID(float ref_vx, float cur_vx, double dt)
{
    /********************************************
     * DEFINE YOUR AMAZING PID CONTROLLER
     * Find Documentation here:
     * https://www.elprocus.com/the-working-of-a-pid-controller/
     ********************************************/

    // Is the controller was set to use it?
    if (m_throttle_ctrl)
    {   
        //Received reference was cero?
        if (ref_vx == 0.0)
        {
            m_vx_int_error = 0.0;
            return 0.0;
        }
        //proportional
        m_vx_prop_ek1 = ref_vx -  cur_vx*dt; //reference says "cur_vx" is a velocity, but it comes from odometry as position. Not sure to include "dt".
        float proportional_th = m_kp_thr * m_vx_prop_ek1;
        //integral
        m_vx_int_error += m_vx_prop_ek1*dt;
        float integral_th = m_ki_thr * m_vx_int_error;
        //derivative
        float derivative_th = m_kd_thr * ((m_vx_prop_ek1 - m_prev_prop_error)/dt);
        
        //update error
        m_prev_prop_error = m_vx_prop_ek1;

        //PID result
        return proportional_th + integral_th + derivative_th;
    }
    else
    {
        return ref_vx;
    }
    /********************************************
     * END CODE
     *  ********************************************/
}

float PIDController::SteeringPID(float ref_wz, float cur_wz, double dt)
{
    /********************************************
     * DEFINE YOUR AMAZING PID CONTROLLER
     * Find Documentation here:
     * https://www.elprocus.com/the-working-of-a-pid-controller/
     *
     * FeedForward:
     * https://zhuanlan.zhihu.com/p/382010500#:~:text=In%20many%20applications,dynamic%20models%20used.
     * "Combined FeedForward and Feedback Control"
     ********************************************/
    
    // Is the controller was set to use it?
    if (m_steering_ctrl)
    {   
        //Received reference was cero?
        if (ref_wz == 0.0)
        {
            m_wz_int_error = 0.0;
            return 0.0;
        }
        //proportional
        m_wz_prop_ek1 = ref_wz -  cur_wz*dt;
        float proportional_st = m_kp_str * m_wz_prop_ek1;

        //integral
        m_wz_int_error += m_wz_prop_ek1*dt;
        float integral_st = m_ki_str * m_wz_int_error;

        //derivative
        float derivative_st = m_kd_str * ((m_wz_prop_ek1 - m_prev_prop_error)/dt);
        
        //update error
        m_prev_prop_error = m_wz_prop_ek1;

        //PID result + FF
        return proportional_st + integral_st + derivative_st + m_kff_str*(ref_wz); //It will be convenient to have a second FF for disturbances. Need to be able to measure them.
    }
    else
    {
        return ref_wz;
    }
    /********************************************
     * END CODE
     *  ********************************************/
}
