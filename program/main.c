//
// Created by Lao·Zhu on 2021/8/20.
//

#include "main.h"

void mdtp_callback_handler(unsigned char pid, unsigned char *data) {
    /* send package back directly for testing */
    mdtp_data_transmit(pid, data);
}

int main(void) {
    /* 4 bits for preemption priority 0 bits for subpriority */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    /* configure systick timer for delay_ms() function */
    systick_config();
    /* configure led gpio */
    led_config();
    /* configure uart for data transmit */
    uart_config();
    /* configure timer1 for pwm output */
    pwm_config();
    /* configure spi0 for encoder communicate */
    spi_config();
    /* configure encoder for foc algorithm */
    encoder_config();
    /* zero the encoder for foc algorithm */
    encoder_zeroing();
    while (1) {
        unsigned char buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        float angle = (float) encoder_get_electronic_angle();
        buffer[0] = float_to_int16(angle) >> 8;
        buffer[1] = float_to_int16(angle) & 0x00ff;
        mdtp_data_transmit(0x00, buffer);
        led_toggle();
        delayms(50);
    }
}
