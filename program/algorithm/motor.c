#include "motor.h"
#include "system.h"
#include "timer.h"
#include "commander.h"
#include "nanoprintf.h"

volatile motor_t motor;

unsigned int monitor_cnt = 0;

void motor_update_cnt(void)
{
    monitor_cnt++;
}

void motor_init(void)
{
    motor.FOC_Struct.vbus = VBUS;
    motor.FOC_Struct.pwmmod = USE_SVPWM;
    motor.FOC_Struct.control_mod = MCT_torque;
    motor.FOC_Struct.rotate_speed = 0;
    motor.phase_sequence = 0;
    motor.foc_parameter_available_flag = 1;
    motor.pid_parameter_available_flag = 1;

    motor.enabled = 0;
    motor.target = 0;
    motor.angle_pid_handler.expect = 0;
    motor.speed_pid_handler.expect = 0;
    motor.FOC_Struct.user_expect = 0;

    motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE; //!< Bit array holding the map of variables the user wants to monitor
    motor.monitor_downsample = 100;
}

void motor_report(void)
{
    if(!motor.monitor_downsample || ((monitor_cnt < motor.monitor_downsample)))
        return;

    monitor_cnt = 0;
    char monitor_start_char = '\0'; //!< monitor starting character 
    char monitor_end_char = '\0'; //!< monitor outputs ending character 
    char monitor_separator = '\t'; //!< monitor outputs separation character
    unsigned int  monitor_decimals = 4; //!< monitor outputs decimal places
    uint8_t printed = 0;
    uint8_t monitor_variables = motor.monitor_variables;

  if(monitor_variables & _MON_TARGET){
    if(!printed && monitor_start_char) print_ch(monitor_start_char);
    npf_printf("%.2f", motor.target);
    printed= true;
  }
  if(monitor_variables & _MON_VOLT_Q) {
    if(!printed && monitor_start_char) print_ch(monitor_start_char);
    else if(printed) print_ch(monitor_separator);
    npf_printf("%.2f", motor.FOC_Struct.user_expect);
    printed= true;
  }
  if(monitor_variables & _MON_VOLT_D) {
    if(!printed && monitor_start_char) print_ch(monitor_start_char);
    else if(printed) print_ch(monitor_separator);
    npf_printf("0.00");
    printed= true;
  }
  // read currents if possible - even in voltage mode (if current_sense available)
  if(monitor_variables & _MON_CURR_Q || monitor_variables & _MON_CURR_D) {
    //todo
    if(monitor_variables & _MON_CURR_Q) {
      if(!printed && monitor_start_char) print_ch(monitor_start_char);
      else if(printed) print_ch(monitor_separator);
      npf_printf("0.00");
      printed= true;
    }
    if(monitor_variables & _MON_CURR_D) {
      if(!printed && monitor_start_char) print_ch(monitor_start_char);
      else if(printed) print_ch(monitor_separator);
      npf_printf("0.00");
      printed= true;
    }
  }
 
  if(monitor_variables & _MON_VEL) {
    if(!printed && monitor_start_char) print_ch(monitor_start_char);
    else if(printed) print_ch(monitor_separator);
    npf_printf("%.2f", motor.FOC_Struct.rotate_speed);
    printed= true;
  }
  if(monitor_variables & _MON_ANGLE) {
    if(!printed && monitor_start_char) print_ch(monitor_start_char);
    else if(printed) print_ch(monitor_separator);
    npf_printf("%.2f", motor.FOC_Struct.mechanical_angle);
    printed= true;
  }
  if(printed){
    if(monitor_end_char) npf_printf("%c\n", monitor_end_char);
    else println("");
  }

}

void motor_enable(void)
{
    /* delay to wait for the motor to respond */
    delayms(1000);

    /* configure timer for foc calculate loop */
    timer2_config();
    if (motor.FOC_Struct.control_mod != TORQUE_LOOP_CONTROL)
        timer13_config();
}

void motor_disable(void)
{
    timer2_disable();
    if (motor.FOC_Struct.control_mod != TORQUE_LOOP_CONTROL)
        timer13_disable();
}
