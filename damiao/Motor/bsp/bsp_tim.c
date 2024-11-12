/*
 * @File         : 
 * @Author       : 
 * @Date         : 
 * @LastEditTime : 
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\newbalD\Bsp\bsp_tim.c
 * @Description  : TIM12用于测量运算时间，TIM6为us延时，TIM5为板载LED，TIM10-CH1为IMU温控PWM，TIM4-CH3为蜂鸣器
 */

#include "bsp_tim.h"
#include "tim.h"

/**
 * @description  : 
 * @return        {*}
 */
//void RunTimStart(void)
//{
//  TIM2 -> CNT = 0x00;
//  HAL_TIM_Base_Start(&htim13);
//}

///**
// * @description  : 
// * @return        {*}
// */
//float RunTimStop(void)
//{
//  float Run_Tim_us;
//  Run_Tim_us = TIM2 -> CNT;
//  HAL_TIM_Base_Stop(&htim13);
//  return Run_Tim_us;
//}

/**
 * @description  : 
 * @param         {uint16_t} us
 * @return        {*}
 */
void Delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);

	while(__HAL_TIM_GetCounter(&htim6) < us);//计数频率1MHz,1次即为1us
	/* Disable the Peripheral */
	HAL_TIM_Base_Stop(&htim6);
}

/**
 * @description  : 
 * @return        {*}
 */
void BuzzerStart(void)
{
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,9000);
}

/**
 * @description  : 
 * @return        {*}
 */
void BuzzerStop(void)
{
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
}

/**
 * @description  : 
 * @return        {*}
 */
void LED_Blue(void)
{
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,5000);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,0);
}

/**
 * @description  : 
 * @return        {*}
 */
void LED_Green(void)
{
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,5000);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,0);
}

/**
 * @description  : 
 * @return        {*}
 */
void LED_Red(void)
{
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,0);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,5000);
}

/**
 * @description  : 
 * @return        {*}
 */
void LED_White(void)
{
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,5000);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,5000);
  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,5000);
}
