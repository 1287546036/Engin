/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef enum
{
  CAN_CHASSIS_ALL_ID = 0x200,
  CAN_AUXILIARY_ALL_ID = 0x1FF,
  motor1 = 0x201,
  motor2 = 0x202,
  motor3 = 0x203,
  motor4 = 0x204,
//	motor1 = 0x205,
//  motor2 = 0x206,
//  motor3 = 0x207,
//  motor4 = 0x208,/////6020
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
typedef struct
{
    struct
    { 
        signed short ch0;
        signed short ch1;
        signed short ch2;
        signed short ch3;
        unsigned char s1;
        unsigned char s2;
        
        unsigned short sw;
    }rc;
}DBUS;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
