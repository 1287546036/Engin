#ifndef __APP_RM_MOTOR_H
#define __APP_RM_MOTOR_H

#include "main.h"
#include "controller.h"
#include "can.h"


#define M3508_AngleMaxOut 16384.0f
#define M3508_SpeedMaxOut 16384.0f

#define M3508_AngleMax_I_Out M3508_AngleMaxOut * 0.1f
#define M3508_SpeedMax_I_Out M3508_SpeedMaxOut * 0.1f

/**
 * @description  : 电机ID，用于收发报文判定
 * @return        {*}
 */
#define M3508_STD_ID 			 0x200




typedef struct
{
    uint16_t encoder; // 转子机械角度(0~8191)
    int16_t last_encoder;

    int16_t speed;      // 转子转速
    int16_t current;    // 转矩电流
    int8_t temperature; // 温度(2006无此项)

    int32_t round; // 转子圈数累计
    int32_t total_encoder;
} RM_MotorMeasure_t;

typedef struct
{
    RM_MotorMeasure_t RM_MotorMeasure;
    float RefAngle;
    uint8_t SpeedCtrl_Log;
    PID_t MotorCtrl_Angle;
    PID_t MotorCtrl_Speed;
} RM_Motor_t;





void Set_RM_MotorCurrent(CAN_HandleTypeDef *hcan, uint32_t DeviceID, int16_t IQ1, int16_t IQ2, int16_t IQ3, int16_t IQ4);
void Get_RM_MotorInfo(RM_MotorMeasure_t *rm_motor_t, uint8_t data[8]);


#endif
