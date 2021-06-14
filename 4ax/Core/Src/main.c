#include "main.h"
#include "init.h"
#include "mpu6050.h"
#include "motor.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"

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

  	//intiialize 
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

  	// ch1 = 850;
  	// HAL_Delay(5000);
  	// ch1 = 0;
  	while(1){

		Read_DMP();
		ToEulerAngles();
	// 	// printf("roll : %.3f pitch : %.3f yaw : %.3f pitch_gryo : %d\n" , roll , pitch , yaw , gyro[1]);
	// 	// pid_control(0.0 , 0.0 , 0.0);
		motor_update();		
	}

}
 


  // void motor_Ctrl(void);
  // xTaskCreate(motor_Ctrl , "car_Ctrl" , 512 , NULL , tskIDLE_PRIORITY+1 , NULL);
  // vTaskStartScheduler();
	// void i2c_detect(void)
	// {
	//   uint8_t devices = 0u;
	//   extern I2C_HandleTypeDef hi2c1;

	//   printf("Searching for I2C devices on the bus...\n");
	//   /* Values outside 0x03 and 0x77 are invalid. */
	//   for (uint8_t i = 0x03u; i < 0x78u; i++)
	//   {
	//     uint8_t address = i << 1u ;
	//     /* In case there is a positive feedback, print it out. */
	//     if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1, address, 3u, 10u))
	//     {
	//       printf("Device found: 0x%02X\n", address);
	//       devices++;
	//     }
	//   }
	//   /* Feedback of the total number of devices. */
	//   if (0u == devices)
	//   {
	//     printf("No device found.\n");
	//   }
	//   else
	//   {
	//     printf("Total found devices: %d\n", devices);
	//   }
	// }
