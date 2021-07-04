#include "main.h"
#include <string.h>
#include <stdio.h>

const char *connectMsg = "AT+CIPSTART=\"UDP\",\"192.168.4.2\",8086\r\n"; 
extern uint8_t rx_Buffer[BUFFER_SIZE];
extern UART_HandleTypeDef huart1;

void ESP_Init(void)
{
	char espOK = 0;
	while(!espOK){

		HAL_UART_Transmit(&huart1 , (uint8_t *)"AT+RST\r\n", 8 , 2000);
		HAL_Delay(100);
		HAL_UART_Transmit(&huart1 , (uint8_t *)"AT+CWMODE=2\r\n", 13 , 2000);
		HAL_Delay(100);
		HAL_UART_Transmit(&huart1 , (uint8_t *)connectMsg , 41 , 2000);
		HAL_Delay(100);
		if(strstr((char *)rx_Buffer , "OK") != NULL){
			printf("Start UDP connection.\n");
			espOK = 1;
		}else{
			printf("ESP8266 Error.\n");
		}
	}
}

void ESP_Send(char *msg , uint8_t len){

	// unsigned int len;
	char cmd[1024];
	// len = strlen(msg);
	snprintf(cmd , 1024 , "AT+CIPSEND=%d\r\n" , len);
	printf("length : %d\n" , len);
	HAL_UART_Transmit(&huart1 , (uint8_t *)cmd , strlen(cmd) , 2000);
	HAL_Delay(5);
	HAL_UART_Transmit(&huart1 , (uint8_t *)msg , len , 2000);
}