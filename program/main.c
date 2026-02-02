/**************************************************************************//**
  \file     main.c
  \brief    miniFOC main source file, The relevant operations and main function
            after unpacking of medium capacity transmission protocol are implemented
            in this document.
  \author   Lao·Zhu
  \version  V1.0.2
  \date     29. October 2021
 ******************************************************************************/

#include "main.h"

/*!
    \brief used to indicate the current status of miniFOC
    \param
      1 calibrate motor phase and sensor offset  \n
      2 enable the motor                         \n
      3 disable the motor                        \n
      4 return current parameters
*/
unsigned char minifoc_fsm_state = 0;

/*!
    \brief main function
*/
int main(void) {
    /* 4 bits for preemption priority 0 bits for subpriority */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    /* configure peripherals */
    led_config();
    uart_config();
    pwm_config();
    spi_config();

    /* configure filter and pid parameters for pid algorithm */
    filter_config();
    pid_config(MCT_velocity);
    motor_init();

    /* configure systick timer for delay_ms() function */
    systick_config();

    /* read all parameters from flash */
    //flash_read_parameters();

    timer2_disable();
    timer13_disable();

    /* correct the mechanical angle zero deviation */
    encoder_zeroing();

    /* automatic phase sequence detection and correction */
    foc_calibrate_phase();

    // flash_write_parameters();
    led_toggle();

    while (1) {
        if(cmd_receive_finish()){
            char * cmd = cmd_get_cmd();
            // npf_printf("---> %s\n", cmd);
            command_handle(cmd);
            cmd_process_finish();
            led_toggle();
        }
        delayms(5);
        motor_report();
        continue;
    }
}
