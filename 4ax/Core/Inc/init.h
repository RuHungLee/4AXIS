#ifndef __INIT_H
#define __INIT_H


void sysInit(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_I2C1_Init(void);
void MX_SPI2_Init(void);

#endif