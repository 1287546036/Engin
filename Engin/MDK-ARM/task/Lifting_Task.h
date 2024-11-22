#include "can.h"
#include "main.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "motor.h"
#ifndef Lifting_Task_H
#define Lifting_Task_H
extern void lifitng_task(void);

extern motor_measure_t motor_lifting[4];

void Lifting_dual_motor_pid(fp32 set_angle,pids *motor_dual_pid,int16_t pid_dual_out[],motor_measure_t *motor1_dual,motor_measure_t *motor2_dual);

void lifting_mode_execute(fp32 rc_lifting,fp32 rc_protract);



#endif

