#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"
#include "stm32f4xx_hal_dma.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024
extern void refereeINIT(TIM_HandleTypeDef *timer);
extern void referee_unpack_fifo_data(void);
extern void refereeReceiveHandler(void);

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

#endif
