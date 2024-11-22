#include "Vofa.h"

#include "struct_typedef.h"
#include "usart.h"

/**
 * @brief Vofa+函数justfloat协议发送函数，使用DMA发送(cubemx内开启对应串口的DMA
 *
 * @param set 下列四个fp32(float)格式变量名称和个数可以自行更改，对应更改ch数组大小即可
 * @param feedback
 * @param target
 * @param speed
 * @param huart 所使用的串口号地址
 */
void JustFloat(fp32 set, fp32 feedback, fp32 target, fp32 speed, UART_HandleTypeDef *huart) {
  // uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};  协议规定的帧尾
  vofa.VofaD.set = set;
  vofa.VofaD.feedback = feedback;
  vofa.VofaD.TargetSpeed1 = target;
  vofa.VofaD.Speed1 = speed;
  vofa.VofaD.tail1 = 0x00;
  vofa.VofaD.tail2 = 0x00;
  vofa.VofaD.tail3 = 0x80;
  vofa.VofaD.tail4 = 0x7f;
  HAL_UART_Transmit_DMA(huart, vofa.ch, 20);
}
