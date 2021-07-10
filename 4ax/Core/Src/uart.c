#include "main.h"
#include "ano.h"
#include "esp.h"
#include <string.h>
#include <stdio.h>

volatile int rx_Len ;  
volatile uint8_t rx_end_flag;

extern int throttle;
extern uint8_t espInit;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern I2C_HandleTypeDef hi2c1;

uint8_t rx_Buffer[BUFFER_SIZE];
uint8_t packetRecved = 0;


#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{

	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}


void USART1_IRQHandler(void)
{
  if(USART1 == huart1.Instance && __HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!= RESET){
    
    HAL_UART_DMAStop(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    rx_Len = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    rx_end_flag = 1;
    HAL_UART_Receive_DMA(&huart1 , rx_Buffer , BUFFER_SIZE);
    
    if(strncmp(rx_Buffer+2 , "+IPD" , 4) == 0){
      
      packetRecved = 1;
    
    }

  }

  //沒有在這裡加 print , ESP 會過不了初始化
  if(!espInit){
    printf("%s\n" , rx_Buffer);
  }

  HAL_UART_IRQHandler(&huart1);
}



void DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);

}

void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}


void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
}

