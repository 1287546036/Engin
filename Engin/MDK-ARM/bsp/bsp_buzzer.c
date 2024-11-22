#include "bsp_buzzer.h"
#include "bsp_tim.h"
void Buzzer(void)// 这个是干嘛的???
{
    BuzzerStart();
    HAL_Delay(80);
    BuzzerStop();
    HAL_Delay(80);
    BuzzerStart();
    HAL_Delay(80);
    BuzzerStop();
    HAL_Delay(80);
    BuzzerStart();
    HAL_Delay(80);
    BuzzerStop();
}