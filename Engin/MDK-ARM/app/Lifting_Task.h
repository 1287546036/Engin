#include "can.h"
#include "main.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "motor.h"
#ifndef Lifting_Task_H
#define Lifting_Task_H



pids motor_pid;

extern motor_measure_t motor_lifting[4];
int16_t pid_lifting[4];


extern motor_measure_t motor_lifting[4];




#endif

