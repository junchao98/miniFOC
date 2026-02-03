#ifndef __MOTOR__
#define __MOTOR__
#include "pid.h"
#include "foc.h"
#include "filter.h"
#include "config.h"

// monitoring bitmap
#define _MON_TARGET 0b1000000 // monitor target value
#define _MON_VOLT_Q 0b0100000 // monitor voltage q value
#define _MON_VOLT_D 0b0010000 // monitor voltage d value
#define _MON_CURR_Q 0b0001000 // monitor current q value - if measured
#define _MON_CURR_D 0b0000100 // monitor current d value - if measured
#define _MON_VEL    0b0000010 // monitor velocity value
#define _MON_ANGLE  0b0000001 // monitor angle value

// default monitor downsample
#define DEF_MON_DOWNSMAPLE 100 //!< default monitor downsample
#define DEF_MOTION_DOWNSMAPLE 0 //!< default motion downsample - disable


typedef struct{
    PID_Structure_t speed_pid_handler; 
    PID_Structure_t angle_pid_handler;
    FOC_Structure_t FOC_Struct;
    Filter_Structure_t velocity_filter;
    float target;
    unsigned int monitor_downsample;
    unsigned char enabled;
    unsigned char motion_downsample;
/*!
    \brief motor phase sequence flag variable
*/
    unsigned char phase_sequence;
    unsigned char pid_parameter_available_flag;
    unsigned char foc_parameter_available_flag;
    unsigned char monitor_variables;
}motor_t;

extern volatile motor_t motor;

void motor_update_cnt(void);
void motor_init(void);
void motor_enable(void);
void motor_disable(void);
void motor_report(void);
#endif // !__MOTOR__
