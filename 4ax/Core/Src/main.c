#include "main.h"
#include "init.h"
#include "mpu6050.h"
#include "motor.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ano.h"

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


void uart_Ctrl(void);

int main(void)
{

 	//initialize gpio , timer , usart , i2c , dma ... 
 	Init();
  	MPU6050_initialize();
  	DMP_Init();

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

		Read_DMP();
		ToEulerAngles();
		motor_update();
		// HAL_Delay(100);
		// printf("pitch : %.3f , gyro_1 : %.3f\n" , pitch , gyro[1]);
		// printf("hello\n");
		// printf("gyro_x : %hi gyro_y : %hi gyro_z : %hi\n" , (short)((float)gyro[0] * 0.0305175) , (short)((float)gyro[1] * 0.0305175) , (short)((float)gyro[2] * 0.0305175)); 		
		// Send_Sensor(1 , 1 , 1 , (short)((float)gyro[0] * 0.0305175) , (short)((float)gyro[1] * 0.0305175) , (short)((float)gyro[2] * 0.0305175) , 1 , 1 , 1);
	}

}
 

