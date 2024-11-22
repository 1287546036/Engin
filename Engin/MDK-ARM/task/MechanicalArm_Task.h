#ifndef MECHANICALARM_TASK
#define MECHANICALARM_TASK

#include "DM_Motor_Ctrl.h"
#include "bsp_tim.h"

extern void mechanicalarm_task(float encoder_angle_set[]);
extern void mechanicalarm_init(void);


void encoder_to_damiao(uint8_t rxBuffer[],float encoder_angle_set[]);

#endif
