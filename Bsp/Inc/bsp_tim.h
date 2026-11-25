#ifndef __BSP_TIM_H__
#define __BSP_TIM_H__

#include "main.h"
#include "tim.h"
#include "receive.h"

void bsp_tim_init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif
