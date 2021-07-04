#include "main.h"
#include "init.h"
#include "mpu6050.h"
#include "motor.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "nrf24.h"
#include "esp.h"
#include "ano.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern I2C_HandleTypeDef hi2c1;
extern float q0 , q1 , q2 , q3 , roll , pitch , yaw , Pitch;
extern int throttle;
extern uint8_t rx_Buffer[1024];

#define CCM_RAM __attribute__((section(".ccmram")))


int main(void)
{

 	//驅動以及網路初始化
 	Init();
  	MPU6050_initialize();
  	DMP_Init();
  	ESP_Init();

  	// 800 : 0% , 1600 : 100%
  	 // intiialize 
  	// ch1 = 1600;
  	// ch2 = 1600;
  	// ch3 = 1600;
  	// ch4 = 1600;
  	// HAL_Delay(3000);
  	// ch1 = 800;
  	// ch2 = 800;
  	// ch3 = 800;
  	// ch4 = 800;
  	// HAL_Delay(3000);

  	ch1 = 800;
  	ch2 = 800;
  	ch3 = 800;
  	ch4 = 800;

  	HAL_Delay(3000);
  	
  	

  	while(1){

  		printf("hello\n");
		Read_DMP();
		ToEulerAngles();
		printf("roll : %.3f , pitch : %.3f , yaw : %.3f\n" , roll , pitch , yaw);
		ANO_DT_Send_Status(roll, pitch , yaw , 0 , 0 , 0);
		// printf("hello\n");
		// HAL_Delay(2000);
		
		// motor_update();
		// HAL_Delay(100);
		// printf("pitch : %.3f , gyro_1 : %.3f\n" , pitch , gyro[1]);
		// printf("hello\n");
		// printf("gyro_x : %hi gyro_y : %hi gyro_z : %hi\n" , (short)((float)gyro[0] * 0.0305175) , (short)((float)gyro[1] * 0.0305175) , (short)((float)gyro[2] * 0.0305175)); 		
		// Send_Sensor(1 , 1 , 1 , (short)((float)gyro[0] * 0.0305175) , (short)((float)gyro[1] * 0.0305175) , (short)((float)gyro[2] * 0.0305175) , 1 , 1 , 1);
	}

}
 

// // Used for not stuck waiting for IRQ
// #define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF

// // Result of packet transmission
// typedef enum {
// 	nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
// 	nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
// 	nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
// 	nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
// } nRF24_TXResult;

// nRF24_TXResult tx_res;

// nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length);



// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
// nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
// 	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
// 	uint8_t status;

// 	// Deassert the CE pin (in case if it still high)
// 	nRF24_CE_L();

// 	// Transfer a data from the specified buffer to the TX FIFO
// 	nRF24_WritePayload(pBuf, length);

// 	// Start a transmission by asserting CE pin (must be held at least 10us)
// 	nRF24_CE_H();

// 	// Poll the transceiver status register until one of the following flags will be set:
// 	//   TX_DS  - means the packet has been transmitted
// 	//   MAX_RT - means the maximum number of TX retransmits happened
// 	// note: this solution is far from perfect, better to use IRQ instead of polling the status
// 	do {
// 		status = nRF24_GetStatus();
// 		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
// 			break;
// 		}
// 	} while (wait--);

// 	// Deassert the CE pin (Standby-II --> Standby-I)
// 	nRF24_CE_L();

// 	if (!wait) {
// 		// Timeout
// 		return nRF24_TX_TIMEOUT;
// 	}

// 	// Check the flags in STATUS register
// 	printf("[");
// 	printf("%d" , status);
// 	printf("] ");

// 	// Clear pending IRQ flags
//     nRF24_ClearIRQFlags();

// 	if (status & nRF24_FLAG_MAX_RT) {
// 		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
// 		return nRF24_TX_MAXRT;
// 	}

// 	if (status & nRF24_FLAG_TX_DS) {
// 		// Successful transmission
// 		return nRF24_TX_SUCCESS;
// 	}

// 	// Some banana happens, a payload remains in the TX FIFO, flush it
// 	nRF24_FlushTX();

// 	return nRF24_TX_ERROR;
// }


  	// nRF24_Init();
	// char nRF24_payload[10];
	// int payload_length = 5;
	// nRF24_RXResult pipe;
	
 //    while (1) {
 //    	//
 //    	// Constantly poll the status of the RX FIFO and get a payload if FIFO is not empty
 //    	//
 //    	// This is far from best solution, but it's ok for testing purposes
 //    	// More smart way is to use the IRQ pin :)
 //    	//
 //    	if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
 //    		// Get a payload from the transceiver
 //    		pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);

 //    		// Clear all pending IRQ flags
	// 		nRF24_ClearIRQFlags();

	// 		// Print a payload contents to UART
	// 		printf("RCV PIPE#");
	// 		// printf("%d" , pipe);
	// 		printf(" PAYLOAD:>");
	// 		printf((char *)nRF24_payload);
	// 		printf("<\r\n");
 //    	}
 //    }

  // 	// The main loop
  //   int i , j = 0;
  //   int payload_length = 5;
  //   char nRF24_payload[10];
  //   while (1) {
  //   	// Prepare data packet
  //   	for (i = 0; i < payload_length; i++) {
  //   		nRF24_payload[i] = 'a'+j++;
  //   		if (j > 0x00000010) j = 0;
  //   	}
  //   	nRF24_payload[payload_length] = 0;

  //   	// Print a payload
  //   	printf("PAYLOAD:>");
  //   	printf("%s" , nRF24_payload);
  //   	printf("< ... TX: ");

  //   	// Transmit a packet
  //   	tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
  //   	switch (tx_res) {
		// 	case nRF24_TX_SUCCESS:
		// 		printf("OK");
		// 		break;
		// 	case nRF24_TX_TIMEOUT:
		// 		printf("TIMEOUT");
		// 		break;
		// 	case nRF24_TX_MAXRT:
		// 		printf("MAX RETRANSMIT");
		// 		break;
		// 	default:
		// 		printf("ERROR");
		// 		break;
		// }
  //   	printf("\r\n");

  //   	// Wait ~0.5s
  //   	Delay_ms(500);
  //   }

