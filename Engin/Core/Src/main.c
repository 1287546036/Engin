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
#include "Chassis_Task.h"
#include "Lifting_Task.h"
#include "Gimbal_Task.h"
#include "referee.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define unit_speed 10000/660.0 
#define angle_ratio 0.1*8191/660.0	
DBUS remoter;
uint8_t dbus_resive[18];
uint8_t	  airpump_tx[1];
/*chassis*/



//pids chassis_motor_pid;

/*lifting*/
//fp32 rc_lifting_angle,rc_protract_angle;
//fp32 set_lifting_angle,set_protract_angle;
//fp32 real_angle_keep;
//pids lifting_motor_pid;
//int16_t pid_lifting[4];

/*debug*/


extern motor_measure_t motor_chassis[4];
pids motor_pid;

int16_t current[4];
int16_t speed[4];


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void airpump_transmit(void);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
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
	////写一个根据电机接收情况亮灯的反馈//先每个任务写�?个初始化,然后�?有初始化写一个函�?
	//�?有pid数据�?类参数整理成�?,分别放在task.h,名称有指向�??
	//缺少各类函数优化,电机过零处理�?//电机,通道值各类方向要便于修改

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

 airpump_tx[0]=1;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//	refereeINIT(TIM6);//??????
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_CAN2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	remote_control_init();
  HAL_UART_Receive_DMA(&huart3,dbus_resive,18);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  can_start();
  HAL_UART_Transmit_IT(&huart1,airpump_tx,1);
  
 /*         debug        */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//com_tar_angle[0]+=motor_chassis[0].real_angle;
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/*            air_pump
		  */
		  
//	USART_SendData(USART1,1);	  
//			airpump_transmit();

//		   HAL_UART_Transmit(&huart1, airpump_tx, sizeof(airpump_tx), 0xffff);
//		  HAL_Delay(100);
//f407tx,f107rx,由于库不同,当前的无法使用
		  //但机械臂可用,同为f407接收
		  
		  
		  
		  
		  
/* 
                      	chassis_task
*/


//chassis_task();


		
//			
///*                      lifitng_task 
//*/


lifitng_task();
///*                     debug
//			                  
//*/

//pidINIT(&motor_pid,PID_POSITION,3,3,0,10000,200);
////fp32 set_angle=5000;//rc_ctrl.rc.ch[1]*8191/660*0.01;


//speed[0]=1600;//PID_calc(&motor_pid,motor_chassis[2].real_angle,motor_chassis[2].real_angle+set_angle);
////current[0]=PID_calc(&motor_pid,motor_chassis[0].real_angle,motor_chassis[0].real_angle+set_angle)+PID_calc(&motor_pid,motor_chassis[0].speed_rpm,0);
//current[2]=PID_calc(&motor_pid,motor_chassis[2].speed_rpm,speed[0]);

//CAN_cmd_chassis(current);


//angle_debug();
  }
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



/*          AIR PUMP        */
//void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
//{
//  /* Check the parameters */
//  assert_param(IS_USART_ALL_PERIPH(USARTx));
//  assert_param(IS_USART_DATA(Data)); 
//    
//  /* Transmit Data */
//  USARTx->DR = (Data & (uint16_t)0x01FF);
//	HAL_Delay(1);
//}
void airpump_transmit(void)
{
	if(rc_ctrl.rc.s[1] == 2)
		airpump_tx[0]=0;
	else
		airpump_tx[0]=1;
}



/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
