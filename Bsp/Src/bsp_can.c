#include "app.h"
#include "bsp_can.h"
#include "can.h"


void bsp_can_init(void){
	CAN_FilterTypeDef filter;
	filter.FilterBank=1;
	filter.FilterMode=CAN_FILTERMODE_IDMASK;
	filter.FilterScale=CAN_FILTERSCALE_16BIT;
	filter.FilterIdHigh=0x0000;
	filter.FilterIdLow=0x0000;
	filter.FilterMaskIdHigh=0x00;//0xFC00;
	filter.FilterMaskIdLow=0x00;//0xFFFF;
	filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	filter.FilterActivation=ENABLE;
	
	HAL_CAN_ConfigFilter(&hcan1, &filter);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);
	
}	


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef		rx_header;
	uint8_t Can_Receive_Data[8]= {0};
		
	HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0, &rx_header, Can_Receive_Data);
	
} 
