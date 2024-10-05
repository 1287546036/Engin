/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "motor.h"
#include "struct_typedef.h"
#include "remote_control.h"
#include "lifting.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define unit_speed 10000/660.0 
#define switch_right 2
DBUS remoter;
uint8_t dbus_resive[18];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern motor_measure_t motor_chassis[4];
extern motor_measure_t motor_lifting[4];
pids motor_pid;
int16_t pid_ID[8];
fp32 set_speed[8];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
//void USART3_IRQHandler(void);
void CAN_cmd_chassis(int16_t motor[]);
void CAN_cmd_lifting(int16_t motor[]);
fp32 PID_calc(pids *pid, fp32 ref, fp32 set);
void Chassis_SolutionForward( fp32 wheel_rpm[],short vx,short vy,short vw);
//////void RC_Unpack(RC_ctrl_t *rc);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

short set_speedx,set_speedy,set_speedw=0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	remote_control_init();
  HAL_UART_Receive_DMA(&huart3,dbus_resive,18);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);/////////////////////////////////////////////////////////////////////////
  can1_start();
	pidINIT(&motor_pid,PID_POSITION,3,0.1,0,10000,1000);
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		////////////////////chassis////////////////////////////////////////////////////
	  	set_speedx = rc_ctrl.rc.ch[1]* unit_speed;
  		set_speedy = rc_ctrl.rc.ch[0]* unit_speed;
//////	  	set_speedx =20;
//////	  	set_speedy =100;
	    Chassis_SolutionForward(set_speed,set_speedx,set_speedy,set_speedw);
   		pid_ID[0] = PID_calc(&motor_pid,motor_chassis[0].speed_rpm,set_speed[0]);
      pid_ID[1] = PID_calc(&motor_pid,motor_chassis[1].speed_rpm,set_speed[1]);
	  	pid_ID[2] = PID_calc(&motor_pid,motor_chassis[2].speed_rpm,set_speed[2]);
	  	pid_ID[3] = PID_calc(&motor_pid,motor_chassis[3].speed_rpm,set_speed[3]);
  	
		if (rc_ctrl.rc.s[0] == switch_right)//mei,ju
		{
		CAN_cmd_chassis(pid_ID);
}
		else
			CAN_cmd_chassis(0);
	 	
//////////////		/////////////////¿ª»·
//////////////		set_speedx = 1000;
//////////////		set_speedy = 100;
//////////////		pid_ID[0] =PID_calc(&motor_pid,motor_chassis[0].speed_rpm,set_speedy);
//////////////	//		pid_ID[0] =	PID_calc(&motor_pid,0,set_speedy);
//////////////	  CAN_cmd_chassis(pid_ID);


///////////////////////////lifting  debug///////////////////////////////////////////////
////      set_speed[4] =500;
////	    set_speed[5] =500;
////   		pid_ID[4] = PID_calc(&motor_pid,motor_lifting[0].speed_rpm,set_speed[4]);
////      pid_ID[5] = PID_calc(&motor_pid,motor_lifting[1].speed_rpm,set_speed[5]);

////      CAN_cmd_lifting(pid_ID);

  }
	
	//pid,rc>motor>chassic,lifting??
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Lifting(pids *motor_pid,uint16_t pid_ID[],motor_measure_t *motor[])
{
	fp32 deviation;
	fp32 set_angle[2];
	fp32 tar_angle[2];
	fp32 tar_speed[2];
	fp32 tar_current[2];

#define angle_ratio 0.001*8191/660.0	

 	set_angle[0] = rc_ctrl.rc.ch[2]*angle_ratio;////same
 	set_angle[1] = rc_ctrl.rc.ch[2]*angle_ratio;
	tar_angle[0] = motor[0]->real_angle +set_angle[0];////same
	tar_angle[1] = motor[1]->real_angle +set_angle[1];
	
	
	tar_speed[0]= PID_calc(motor_pid,motor[0]->real_angle,tar_angle[0]);////same
	tar_speed[1]= PID_calc(motor_pid,motor[1]->real_angle,tar_angle[1]);
	
	tar_current[0] = PID_calc(motor_pid,motor[0]->speed_rpm,tar_speed[0]);
  tar_current[1] = PID_calc(motor_pid,motor[1]->speed_rpm,tar_speed[1]);////deffirent
	deviation = 0.5f*(tar_speed[0] - tar_speed[1]);
	
	pid_ID[4] = PID_calc(motor_pid,motor[0]->real_current,tar_current[0]);//
  pid_ID[5] = PID_calc(motor_pid,motor[1]->real_current,tar_current[1])-deviation;
}
/*
    float speed_diff = motor1->speed - motor2->speed;

    float sync_adjustment = 0.5f * speed_diff; // ????????,????????????

    motor1->pid_output = pid_controller(setpoint_speed, motor1->speed, params);
    motor2->pid_output = pid_controller(setpoint_speed, motor2->speed, params) - sync_adjustment; // ??????????
*/


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
