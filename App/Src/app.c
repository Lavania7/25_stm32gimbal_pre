#include "app.h"
#include <stdio.h>
#include "M3508.h"
#include "gimbal.h"
#include "receive.h"


void app_exe()
{
    gimbal_get_Kdata(&gimbal_data, &K230_datapack);
    gimbal_find_target(&gimbal_data);
    gimbal_move(&gimbal_data, &M6020_Motor);
    
}
