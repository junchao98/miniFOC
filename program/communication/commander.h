#ifndef __COMMANDER_H__
#define __COMMANDER_H__

#include "uart.h"
#include <stdint.h>
#include <stddef.h>

#define true 1
#define false 0
void command_handle(char *user_input);
#define npf_printf(...) npf_pprintf(&uart_sendbyte_ctx, NULL, __VA_ARGS__)
void print_ch(char ch);
void println(char *message);
#endif
