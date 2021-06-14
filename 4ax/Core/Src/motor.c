#include "main.h"
#include "motor.h"

extern TIM_HandleTypeDef htim3 , htim4;


int throttle;

void motor_Init()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); 
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); 
}


// int blt_timer_ms = 0;
// void motor_Ctrl(void){
  
//   while(1)
//   {

//     if(blt_timer_ms >= 1000){ mt_Forward(0 , 0 , 0 , 0); }

//     if(rx_end_flag == 1){
      
//       blt_timer_ms = 0;
//       rx_end_flag = 0;
//       HAL_UART_Transmit(&huart2,rx_Buffer,rx_Len,0xffff);
//       switch(rx_Buffer[0]){
        
//         case 'F': mt_Forward(1000 , 1000 , 1000 , 1000); break;

//         case 'B': mt_Backward(1000 , 1000 , 1000 , 1000); break;
        
//         case 'R': mt_Forward(1000 , 0 , 1000 , 0); break;
        
//         case 'L': mt_Forward(0 , 1000 , 0 , 1000); break;

//         default: mt_Forward(0 , 0 , 0 , 0); break;
//       }
//     }
//     vTaskDelay(200);
//   }
// }