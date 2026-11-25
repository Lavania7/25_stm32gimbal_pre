#include "app.h"
#include "bsp_uart.h"
#include "usart.h"


void bsp_uart_init()
{
    pack_receive(&K230_datapack, &huart1);  // 启动初始接收
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    { 
        pack_rx_handler(&K230_datapack);
        pack_receive(&K230_datapack, &huart1);
    }
}

// 错误回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        pack_error_handler(&K230_datapack);
        pack_receive(&K230_datapack, &huart1);
    }
}


