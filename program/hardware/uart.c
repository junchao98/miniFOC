/**************************************************************************//**
  \file     uart.c
  \brief    this file contains the code implementation of UART interface
            initialization function and medium capacity transmission protocol
            transceiver function.
  \author   Lao·Zhu
  \version  V1.0.2
  \date     29. October 2021
 ******************************************************************************/

#include "uart.h"
#include "config.h"
#include "gd32f1x0.h"
#include "system.h"

/*!
    \brief medium capacity transport protocol receive state variable
           0 idle state and waiting for start of package           \n
           1 receive status trying to receive a complete packet.   \n
           2 end status processing the received data
*/
volatile static unsigned char mdtp_receive_status = 0;
/*!
    \brief medium capacity transport protocol receive character counter
*/
volatile static unsigned char mdtp_receive_number_counter = 0;
/*!
    \brief medium capacity transport protocol receive buffer array
*/

typedef enum {
    UART_CMD_IDLE = 0,        // 空闲状态，等待命令接收
    UART_CMD_RECEIVING,       // 正在接收命令
    UART_CMD_RECEIVED,        // 命令接收完成
    UART_CMD_PROCESSING,      // 正在处理接收到的命令
    UART_CMD_PROCESSED,       // 命令处理完成
    UART_CMD_ERROR            // 命令接收或处理过程中出现错误
} uart_cmd_st;

static unsigned char mdtp_receive_data_buffer[10] = {0};

#define CMD_BUFFER_SIZE 32
static unsigned char cmd_buffer[CMD_BUFFER_SIZE] = {0};
uint8_t cmd_status = UART_CMD_IDLE;
uint8_t cmd_idx = 0;

void cmd_receive_hander(uint8_t data)
{
    if(cmd_status == UART_CMD_IDLE || cmd_status == UART_CMD_PROCESSED){
        cmd_status = UART_CMD_RECEIVING;
        cmd_buffer[cmd_idx++] = data;
    }else if (cmd_status == UART_CMD_RECEIVING){
        cmd_buffer[cmd_idx++] = data;
        if(data == '\n'){
            cmd_status = UART_CMD_RECEIVED;
            cmd_idx = 0;
        }
        if(cmd_idx == CMD_BUFFER_SIZE){
            cmd_status = UART_CMD_ERROR;
            cmd_idx = 0;
        }
    }else if(cmd_status == UART_CMD_ERROR){
        if(data == '\n'){
            cmd_status = UART_CMD_IDLE;
        }
    }
}

uint8_t cmd_receive_finish(void)
{
    return cmd_status == UART_CMD_RECEIVED;
}

void cmd_process_finish(void)
{
    cmd_status = UART_CMD_PROCESSED;
}

char* cmd_get_cmd(void)
{
    return  cmd_buffer;
}

void uart_sendbyte_ctx(int x, void *ctx)
{
    usart_data_transmit(USART0, (uint8_t) x);
    while (RESET ==usart_flag_get(USART0, USART_FLAG_TBE));
}
/*!
    \brief     UART send single byte macro
    \param[in] x: byte to be send from UART
*/
void uart_sendbyte(uint8_t x)
{
    usart_data_transmit(USART0, (uint8_t) x);
    while (RESET ==usart_flag_get(USART0, USART_FLAG_TBE));
}
/*!
    \brief configure uart0 periph and its gpios
*/
void uart_config(void) {
    /* UART interrupt configuration */
    nvic_irq_enable(USART0_IRQn, UART_PRIORITY, 0);

    /* enable GPIO clock and UART clock*/
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to UARTx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

    /* configure UART Tx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);

    /* configure UART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);

    /* UART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, UART_BAUDRATE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_enable(USART0);

    /* enable UART RBNE interrupt */
    usart_interrupt_enable(USART0, USART_INT_RBNE);
}
