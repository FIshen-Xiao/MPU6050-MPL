#ifndef __S_UART_H
#define __S_UART_H

#define RXBUFFERSIZE  256     //最大接收字节数

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stdarg.h"
#include "stdio.h"
#include "main.h"
#include "usart.h"
#include "string.h"
#include "stm32f4xx_hal_uart.h"

void UART_printf(UART_HandleTypeDef *uartbase, const char *fmt, ...);

#endif

