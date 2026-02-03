/**************************************************************************//**
  \file     pid.c
  \brief    PID algorithm implementation source code
  \author   Lao·Zhu
  \version  V1.0.2
  \date     29. October 2021
 ******************************************************************************/

#include "pid.h"
#include "motor.h"
#include "fast_math.h"
#include "system.h"

PID_Structure_t* pid_get_speed_pid(void)
{
    return &motor.speed_pid_handler;
}

PID_Structure_t* pid_get_angle_pid(void)
{
    return &motor.angle_pid_handler;
}
/*!
    \brief configure pid loop parameters
*/
void pid_config(unsigned char mode) {
    /* clear the value of the PID handler */
    user_memset((void *) &motor.speed_pid_handler, 0x00, sizeof(PID_Structure_t));
    user_memset((void *) &motor.angle_pid_handler, 0x00, sizeof(PID_Structure_t));

    /* update the PID closed loop flag byte */
    motor.FOC_Struct.control_mod = mode;

    /* set maximum and minimum output torque */
    motor.speed_pid_handler.maximum = 1.0f;
    motor.speed_pid_handler.minimum = -1.0f;
    motor.speed_pid_handler.sum_maximum = 500;

    motor.speed_pid_handler.kp = 0.3f;
    motor.speed_pid_handler.ki = 0.01f;
    motor.speed_pid_handler.kd = 0.00f;

    /* set maximum and minimum output speed */
    motor.angle_pid_handler.maximum = 100.0f;
    motor.angle_pid_handler.minimum = -100.0f;
    motor.angle_pid_handler.sum_maximum = 100;

    motor.angle_pid_handler.kp = 3.0f;
    motor.angle_pid_handler.ki = 0.0f;
    motor.angle_pid_handler.kd = 0.0f;
}

/*!
    \brief     calculate result using sampling value
    \param[in] pid_handler: PID data handler
    \param[in] collect: sampled data
    \retval    calculated output value of PID controller
*/
float pid_calculate_result(PID_Structure_t *pid_handler, float collect) {
    /* calculate PID error value */
    float current_result, error = pid_handler->expect - collect;

    /* calculate the integral and realize anti integral saturation */
    pid_handler->summary = pid_handler->summary + error;
    // pid_handler->summary = 1;
    // pid_handler->summary = fast_constrain(pid_handler->summary, -pid_handler->sum_maximum, pid_handler->sum_maximum);

    /* calculate PID output value */
    current_result = pid_handler->kp * error + pid_handler->ki * pid_handler->summary
        + pid_handler->kd * (error - pid_handler->last_error);

    /* Update last time error of PID to calculate the differential term */
    pid_handler->last_error = error;

    /* Implementation of output limiting algorithm */
    current_result = fast_constrain(current_result, pid_handler->minimum, pid_handler->maximum);
    return current_result;
}
