#include "app.h"
#include "bsp_tim.h"



void bsp_tim_init()
{
  	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim3);
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim ->Instance == TIM4){
		app_exe();
	}
	
	else if(htim ->Instance == TIM3){
		
	}
	
}
