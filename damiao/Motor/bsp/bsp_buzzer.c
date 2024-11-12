#include "bsp_buzzer.h"

void Buzzer(void)
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