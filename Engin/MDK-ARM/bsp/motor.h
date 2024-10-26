#include "can.h"
#include "main.h"
#include "struct_typedef.h"
#ifndef motor_H
#define motor_H

void can_start(void);
void CAN_cmd_chassis(int16_t motor[]);
void CAN_cmd_lifting(int16_t motor[]);
void CAN_cmd_gimbal(int16_t motor[]);
typedef enum
{
  CAN_FIRST_ALL_ID = 0x200,
  CAN_SECOND_ALL_ID = 0x1FF,
  motor1 = 0x201,
  motor2 = 0x202,
  motor3 = 0x203,
  motor4 = 0x204,
  motor5 = 0x205,
  motor6 = 0x206,
  motor7 = 0x207,
  motor8 = 0x208,
}can_msg_id;
 typedef struct 
{
    uint16_t angle_value;
    int16_t speed_rpm;
    int16_t real_current;
    uint8_t temperate;
    int16_t real_angle;
}motor_measure_t;

#endif
