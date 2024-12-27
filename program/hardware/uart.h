/**************************************************************************//**
  \file     uart.h
  \brief    this is the header file of uart.c.
  \author   Lao·Zhu
  \version  V1.0.1
  \date     10. October 2021
 ******************************************************************************/

#ifndef MINIFOC_HARDWARE_UART_H_
#define MINIFOC_HARDWARE_UART_H_

#include <stdint.h>

void uart_sendbyte(uint8_t x);
void uart_sendbyte_ctx(int x, void *ctx);
void uart_config(void);
void mdtp_data_transmit(unsigned char pid, const unsigned char *buffer);
void mdtp_receive_handler(unsigned char data);
void mdtp_callback_handler(unsigned char pid, const unsigned char *data);

void cmd_receive_hander(uint8_t data);
uint8_t cmd_receive_finish(void);
void cmd_process_finish(void);
char* cmd_get_cmd(void);

#endif // MINIFOC_HARDWARE_UART_H_
