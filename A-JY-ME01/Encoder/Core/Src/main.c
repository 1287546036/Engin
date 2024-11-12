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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t encoder_resive;
typedef struct
{
	uint8_t id;
	uint32_t get_data;
	float angle;
}encoder_back_t;

encoder_back_t encoder_A_back;
encoder_back_t encoder_B_back;
encoder_back_t encoder_C_back;
encoder_back_t encoder_D_back;
//typedef _packed struct 
//{ 
//uint8_t data[x]; 
//}custom_robot_data_t;
uint32_t angle_count;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_RX_BUFFER_SIZE 10 // 定义最大接收缓冲区大小，根据实际需要调整

uint8_t rxBuffer[MAX_RX_BUFFER_SIZE];  // 定义接收数据的数组
uint32_t rxBufferIdx = 0;  // 定义接收数据的索引


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
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//中断使能DMA
	HAL_UART_Receive_IT(&huart6, &rxBuffer[rxBufferIdx], MAX_RX_BUFFER_SIZE); 
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  
//	  
//	  if(1==rx_done)//检测数据是否接收完成
//{
//	idle_detect=0;//清零标志位
//	//此处添加相应的数据处理代码吧

//}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
/*
//接收四个编码器,并处理为角度
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart6)  // 假设是串口6的接收中断
  {
    rxBuffer[rxBufferIdx++] = huart->Instance->DR;  // 将接收到的数据存入数组，并更新索引

    if (rxBufferIdx >= MAX_RX_BUFFER_SIZE)
    {
      rxBufferIdx = 0;  // 如果超过了数组大小，可以选择重置索引或者其他处理方式
    }
	
	if(rxBuffer[0]==0x01&&rxBuffer[1]==0x03&&rxBuffer[2]==0x04)
{
	encoder_A_back.id = rxBuffer[0];
	encoder_A_back.get_data= rxBuffer[4]  << 16  |
							(( rxBuffer[5] << 8 )|( rxBuffer[6] ));
	angle_count=encoder_A_back.get_data;
	encoder_A_back.angle =(float)angle_count/262144*360;
}
	if(rxBuffer[0]==0x02&&rxBuffer[1]==0x03&&rxBuffer[2]==0x04)
{
	encoder_B_back.id = rxBuffer[0];
	encoder_B_back.get_data= rxBuffer[4]  << 16  |
							(( rxBuffer[5] << 8 )|( rxBuffer[6] ));
	angle_count=encoder_B_back.get_data;
	encoder_B_back.angle =(float)angle_count/262144*360;
}
	if(rxBuffer[0]==0x03&&rxBuffer[1]==0x03&&rxBuffer[2]==0x04)
{
	encoder_C_back.id = rxBuffer[0];
	encoder_C_back.get_data= rxBuffer[4]  << 16  |
							(( rxBuffer[5] << 8 )|( rxBuffer[6] ));
	angle_count=encoder_C_back.get_data;
	encoder_C_back.angle =(float)angle_count/262144*360;
}
	if(rxBuffer[0]==0x04&&rxBuffer[1]==0x03&&rxBuffer[2]==0x04)
{
	encoder_D_back.id = rxBuffer[0];
	encoder_D_back.get_data= rxBuffer[4]  << 16  |
							(( rxBuffer[5] << 8 )|( rxBuffer[6] ));
	angle_count=encoder_D_back.get_data;
	encoder_D_back.angle =(float)angle_count/262144*360;
}
    HAL_UART_Receive_IT(&huart6, &rxBuffer[rxBufferIdx], MAX_RX_BUFFER_SIZE);  // 重新启动接收
  }
}



/*
//合并数据,发送
*/

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
