/**************************************************************************//**
  \file     pid.h
  \brief    this is the header file of pid.c, which defines the structure
            of PID algorithm and closed-loop state macro.
  \author   Lao·Zhu
  \version  V1.0.1
  \date     9. October 2021
 ******************************************************************************/

#ifndef MINIFOC_ALGORITHM_PID_H_
#define MINIFOC_ALGORITHM_PID_H_

#include <stdint.h>

/*! \brief torque loop control mode */
#define TORQUE_LOOP_CONTROL     1
/*! \brief speed loop control mode */
#define SPEED_LOOP_CONTROL      2
/*! \brief angle loop control mode */
#define ANGLE_LOOP_CONTROL      3

/*!
  \struct PID_Structure_t
  \brief  structure of PID algorithm
 */
typedef struct {
    float kp;           ///< proportional term coefficient in PID
    float ki;           ///< integral term coefficient in PID
    float kd;           ///< differential term coefficient in PID
    float summary;      ///< value of integral term in PID
    float expect;       ///< user expectations in PID
    float maximum;      ///< maximum output in PID
    float minimum;      ///< minimum output in PID
    float sum_maximum;  ///< maximum of anti saturation integral in PID
    float last_error;   ///< error value of previous calculation in PID
} PID_Structure_t;

enum MotionControlType {
  MCT_torque            = 0x00,     //!< Torque control
  MCT_velocity          = 0x01,     //!< Velocity motion control
  MCT_angle             = 0x02,     //!< Position/angle motion control
  MCT_velocity_openloop = 0x03,
  MCT_angle_openloop    = 0x04
};

void pid_config(unsigned char mode);
float pid_calculate_result(PID_Structure_t *pid_handler, float collect);
PID_Structure_t* pid_get_speed_pid(void);
PID_Structure_t* pid_get_angle_pid(void);

#endif //MINIFOC_ALGORITHM_PID_H_
