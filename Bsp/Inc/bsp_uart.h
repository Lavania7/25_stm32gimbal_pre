#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "main.h"


void bsp_uart_init(void);

extern DMA_HandleTypeDef hdma_usart6_rx;
extern uint8_t uart6_rx_buffer[100];

#endif
