#include "main.h"
#include "ano.h"
#include "esp.h"
#include <string.h>
#include <stdio.h>

volatile int rx_Len ;  
volatile uint8_t rx_end_flag;

uint8_t rx1_Buffer[BUFFER_SIZE];
uint8_t rx4_Buffer[BUFFER_SIZE];
uint8_t rx1Recv;
uint8_t rx4Recv;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern I2C_HandleTypeDef hi2c1;

extern uint8_t wifi_rx_Buffer[1024];
extern uint8_t wifi2_rx_Buffer[1024];
extern uint8_t espInit;



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
    HAL_UART_Receive_DMA(&huart1 , rx1_Buffer , BUFFER_SIZE);
    
    if(strncmp(rx1_Buffer+2 , "+IPD" , 4) == 0){
      
      memcpy(wifi2_rx_Buffer , rx1_Buffer+12 , 100);
      rx1Recv = 1;
    
    }

  }

  //沒有在這裡加 print , ESP 會過不了初始化
  if(!espInit){
    printf("%s\n" , rx1_Buffer);
  }

  HAL_UART_IRQHandler(&huart1);

}

void UART4_IRQHandler(void)
{

  if(UART4 == huart4.Instance && __HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE)!= RESET){
    
    HAL_UART_DMAStop(&huart4);
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);
    rx_Len = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
    rx_end_flag = 1;
    HAL_UART_Receive_DMA(&huart4 , rx4_Buffer , BUFFER_SIZE);
    
    if(rx4_Buffer[0] == '\xaa' && rx4_Buffer[1] == '\xaf'){

      memcpy(wifi_rx_Buffer , rx4_Buffer , 100);
      rx4Recv = 1;
    
    }
    
  }
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

void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
}

void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
}

