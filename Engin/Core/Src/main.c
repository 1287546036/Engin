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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Chassis_Task.h"
#include "Lifting_Task.h"
#include "Gimbal_Task.h"
#include "MechanicalArm_Task.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define unit_speed 10000/660.0 
#define angle_ratio 0.1*8191/660.0	
DBUS remoter;
uint8_t dbus_resive[18];

/*chassis*/

/*lifting*/

/*debug*/


//extern motor_measure_t motor_chassis[4];
//pids motor_pid;

//int16_t current[4];
//int16_t speed[4];

 
static uint8_t rxBuffer[MAX_RX_BUFFER_SIZE];  // 定义接收数据的数组
uint32_t rxBufferIdx = 0;  // 定义接收数据的索引

static float encoder_angle_set[4];

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

void LED_RX_Status_display(void);


  uint8_t	airpump_tx[] ="STM32F407xx" ;
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
// 等待优化列表
// DWT 延时引入
//	 学习引入pid优化 
// 写一个根据电机接收情况亮灯的反馈
// 先每个任务写个初始化,然后所有初始化封装为一个
// pid 参数整理,分别放在task.h,名称有指向
// 缺少各类函数优化,电机过零处理�?
// 电机,通道值各类方向要便于修改
// 状态机优化完整,宏或其他方法

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
	mechanicalarm_init();

	remote_control_init();
    can_start();


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
  MX_USART1_UART_Init();
  MX_CAN2_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

// HAL_UART_Transmit(&huart1, airpump_tx, sizeof(airpump_tx),500);

//  HAL_UART_Receive_DMA(&huart3,dbus_resive,18);
//  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    HAL_UART_Receive_IT(&huart6,rxBuffer,MAX_RX_BUFFER_SIZE);
 /*         debug        */
//  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		  
/*    
		  air_pump
*/
		  
////	if(rc_ctrl.rc.s[1] == 2){
// HAL_UART_Transmit(&huart1, airpump_tx, sizeof(airpump_tx),500);//发送没有问题,注意rx,tx
////	}
//		  
//		  
		  
		  
/* 
                      	chassis_task
*/


//chassis_task();


		
//			
///*                      lifitng_task 
//*/


//lifitng_task();


/*						MechanicalArm_Task
*/
	HAL_UART_Receive(&huart6,rxBuffer,MAX_RX_BUFFER_SIZE,0xffff);
	HAL_UART_Receive_IT(&huart6,rxBuffer,MAX_RX_BUFFER_SIZE);
	encoder_to_damiao(rxBuffer,encoder_angle_set);


	mechanicalarm_task(encoder_angle_set);


///*                     debug
//			                  
//*/

//JustFloat(0, 0, motor_lifting[1].real_angle,motor_lifting[0].real_angle, &huart6);//vofa

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




/*		debug_damiao_rx		*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart6)  // 假设是串口6的接收中断
  {
    rxBuffer[rxBufferIdx++] = huart->Instance->DR;  // 将接收到的数据存入数组，并更新索引

    if (rxBufferIdx >= MAX_RX_BUFFER_SIZE)
    {
      rxBufferIdx = 0;  // 如果超过了数组大小，可以选择重置索引或者其他处理方式
    }
  }

}


/////*        亮灯显示接收状态              */
//void LED_RX_Status_display(void)//有问题,灯点不亮
//{
//	//每个电机分开写if,怎么判断值齐不齐?//不需要判断,都有就不会进if
//	//缺那几个id的就连续闪几下,隔几秒再闪几下
//	//不同部分颜色分开,分先后
//	uint8_t id,n;
//	for(id=0;id<=3;id++)
//	{
//		if(motor_chassis[id].temperate == NULL)
//		{	for(n=0;n<=id;n++)
//			{
//				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);
//				HAL_Delay(500);
//				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_RESET);
//				HAL_Delay(500);
//				
//				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET);
//				HAL_Delay(500);
//				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_RESET);
//				HAL_Delay(500);
//				
//				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET);
//				HAL_Delay(500);
//				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET);
//				HAL_Delay(500);
//			}
//		HAL_Delay(2000);
//		}
//	}
//}

/* USER CODE END 4 */

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
