#ifndef __RECEIVE_H__
#define __RECEIVE_H__
#include "usart.h"
#include "M3508.h"

// 定义常量
#define PACKET_SIZE     7
#define PACKET_Header   0xDD
#define PACKET_Tail     0xFF
#define BLOCKING_TIMEOUT 20  // 阻塞接收超时时间(ms)

typedef enum __pack_state {
    pack_Waiting,     // 等待帧头状态
    pack_Receiving,   // 接收数据中状态
    pack_OK,          // 接收/处理完成
    pack_Error        // 接收/处理错误
} pack_state;

typedef struct __datapack_TypeDef // K230数据
{
    uint16_t PACK_SIZE;     // 数据包大小
    uint8_t header;         // 帧头
    uint8_t tail;           // 帧尾
    
    uint8_t rx_buff[1];     // 单字缓冲区

    uint8_t last_pack[PACKET_SIZE]; // 上一组数据包
    uint8_t this_pack[PACKET_SIZE]; // 当前数据包
    uint16_t data_index;     // 当前数据位置（改为16位以防大数据包）
    
    uint16_t rx_receive_len; // 当前接收长度（改为16位）
    pack_state HTC_flag;    // 帧头检测状态
    pack_state pack_received; // 包接收完成标志
    
} datapack_TypeDef;

void pack_init(datapack_TypeDef* pack);
void pack_receive(datapack_TypeDef* pack, UART_HandleTypeDef* huart);
void pack_deal_byte(datapack_TypeDef* pack, uint8_t rx_byte);
void pack_rx_handler(datapack_TypeDef* pack);
void pack_error_handler(datapack_TypeDef* pack);

extern datapack_TypeDef K230_datapack;

#endif
