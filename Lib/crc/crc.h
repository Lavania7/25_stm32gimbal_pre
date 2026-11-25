#ifndef __CRC_H__
#define __CRC_H__
 
 
#include "main.h" 
uint8_t crc8(const uint8_t *data, size_t length);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif
