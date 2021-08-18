#include "main.h"
#include <string.h>
#include <stdio.h>

const char *udpClient = "AT+CIPSTART=0,\"UDP\",\"192.168.4.3\",8086\r\n"; 
const char *recvPort = "AT+CIPSTART=1,\"UDP\",\"192.168.4.1\",8086,8086,0\r\n";

extern uint8_t rx1_Buffer[BUFFER_SIZE];
extern UART_HandleTypeDef huart1;
uint8_t espInit = 0;

void ESP_Init(void)
{
	
	while(!espInit){
		HAL_UART_Transmit(&huart1 , (uint8_t *)"ATE0\r\n", 6 , 2000);
		HAL_Delay(100);
		HAL_UART_Transmit(&huart1 , (uint8_t *)"AT+RST\r\n", 8 , 2000);
		HAL_Delay(100);
		HAL_UART_Transmit(&huart1 , (uint8_t *)"AT+CWMODE=2\r\n", 13 , 2000);
		HAL_Delay(100);
		HAL_UART_Transmit(&huart1 , (uint8_t *)"AT+CIPMUX=1\r\n", 13 , 2000);
		HAL_Delay(100);
		HAL_UART_Transmit(&huart1 , (uint8_t *)udpClient , 41 , 2000);
		HAL_Delay(100);
		HAL_UART_Transmit(&huart1 , (uint8_t *)recvPort , 47 , 2000);
		HAL_Delay(100);
		if(strstr((char *)rx1_Buffer , "OK") != NULL){
			printf("Start UDP connection.\n");
			espInit = 1;
		}else{
			printf("ESP8266 Error.\n");
		}
	}
}

void ESP_Send(char *msg , uint8_t len){

	// unsigned int len;
	char cmd[1024];
	// len = strlen(msg);
	snprintf(cmd , 1024 , "AT+CIPSEND=0,%d\r\n" , len);
	// printf("length : %d\n" , len);
	HAL_UART_Transmit(&huart1 , (uint8_t *)cmd , strlen(cmd) , 2000);
	HAL_Delay(0.1);
	HAL_UART_Transmit(&huart1 , (uint8_t *)msg , len , 2000);
}