#include <string.h>
#include <stdio.h>

#include "main.h"
#include "post.h"
#include "init.h"
#include "mpu6050.h"
#include "motor.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "esp.h"
#include "ano.h"
#include "bmp.h"

#define CCM_RAM __attribute__((section(".ccmram")))

extern float roll , pitch , yaw;
extern float height; //cm
extern pst pid_roll , pid_pitch , pid_yaw , pid_height;
extern int throttle;
extern bmp_t bmp;
extern UART_HandleTypeDef huart2 , huart3 , huart4;
extern uint8_t rx1_Buffer[1024];
extern uint8_t rx4_Buffer[1024];
extern uint8_t rx1Recv;
extern uint8_t rx4Recv;

uint8_t tf_rx_Buffer[1024]; 
uint8_t wifi_rx_Buffer[1024];

post Qpost = {

    .throttle = 0,
    .roll = 0,
    .pitch = 0,
    .yaw = 0

};

void packetHandler(void);
void PIDupdate(char *);
void POSEupdate(char *);
void sendStatus(void);

int main(void)
{

   	//驅動以及網路初始化
   	sysInit();
  	MPU6050_initialize();
  	DMP_Init();
  	ESP_Init();

    // 1ms : MPU9250數據接收
    // 2ms : 三軸 PID 控制
    // 20ms : 遙控數據接收
    // 30ms : TF02 高度數據接收 , 高度 PID 控制
    // 50ms : 觀測資料傳送

    xTaskCreate(Read_DMP , "MPU9250數據接收" , 1024 , NULL , 4 , NULL);
    xTaskCreate(AngPIDController , "三軸 PID 控制" , 1024 , NULL , 1 , NULL);
    xTaskCreate(packetHandler , "無線通訊數據接收" , 1024 , NULL , 2 , NULL);
    xTaskCreate(sendStatus , "傳送資料至地面站" , 1024 , NULL , 3 , NULL);
    // xTaskCreate(heightPIDController , "高度 PID 控制" , 1024 , NULL , 4 , NULL);
    
    vTaskStartScheduler();

}

void sendStatus(void){

  while(1){

    ANO_DT_Send_Status(roll , pitch , yaw , height , 0 , 0);

    vTaskDelay(50);

  }

}

void heightPIDController(void){

  while(1){
    
    if (__HAL_UART_GET_FLAG(&huart3 , UART_FLAG_RXNE) == SET){

      HAL_UART_Receive(&huart3 , tf_rx_Buffer , 10 , 200);

      if(tf_rx_Buffer[0] == '\x59' && tf_rx_Buffer[1] == '\x59'){
        
        height = (float)tf_rx_Buffer[3];
      
      }

    }

    vTaskDelay(30);
  }

}

void packetHandler(void)
{

  while(1){

    if (rx4Recv){

      if(wifi_rx_Buffer[0] == '\xaa' && wifi_rx_Buffer[1] == '\xaf'){

        char *ptr =  wifi_rx_Buffer;
        
        ptr += 2;

        switch(ptr[0]){
          
          case '\x10':

            PIDupdate(ptr+1);

          case '\x03':

            POSEupdate(ptr+1);
        }

      }

    }

    rx4Recv = 0;

    vTaskDelay(20);

  }

}


void PIDupdate(char *packet){

  char *ptr;
  uint8_t sum , ck , i;
  float p1 , i1 , d1 , p2 , i2 , d2 , p3 , i3 , d3 , p4 , i4 , d4;
  float p1_rate , i1_rate , d1_rate , p2_rate , i2_rate , d2_rate , p3_rate , i3_rate , d3_rate , p4_rate , i4_rate , d4_rate;

  sum = 0;
  ptr = packet;
  
  
  ptr = ptr + 1;
  p1 = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  i1 = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  d1 = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  p2 = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  i2 = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  d2 = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  p3 = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  i3 = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  d3 = *(uint16_t *)ptr/1000.0;

  ptr = ptr + 2;
  p4 = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  i4 = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  d4 = *(uint16_t *)ptr/1000.0;

  ptr = ptr + 2;
  p1_rate = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  i1_rate = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  d1_rate = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  p2_rate = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  i2_rate = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  d2_rate = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  p3_rate = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  i3_rate = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  d3_rate = *(uint16_t *)ptr/1000.0;

  ptr = ptr + 2;
  p4_rate = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  i4_rate = *(uint16_t *)ptr/1000.0;
  
  ptr = ptr + 2;
  d4_rate = *(uint16_t *)ptr/1000.0;

  ptr = ptr + 2;
  ck = *(char *)ptr;

  for(i = 1 ; i <= ptr - packet - 1 ; i++){
    sum += packet[i];
  }

  if(ck == sum){
    
    pid_roll.kp = p1;
    pid_roll.ki = i1;
    pid_roll.kd = d1;
    pid_pitch.kp = p2;
    pid_pitch.ki = i2;
    pid_pitch.kd = d2;
    pid_yaw.kp = p3;
    pid_yaw.ki = i3;
    pid_yaw.kd = d3;
    pid_height.kp = p4;
    pid_height.ki = i4;
    pid_height.kd = d4;


    pid_roll.kp_rate = p1_rate;
    pid_roll.ki_rate = i1_rate;
    pid_roll.kd_rate = d1_rate;
    pid_pitch.kp_rate = p2_rate;
    pid_pitch.ki_rate = i2_rate;
    pid_pitch.kd_rate = d2_rate;
    pid_yaw.kp_rate = p3_rate;
    pid_yaw.ki_rate = i3_rate;
    pid_yaw.kd_rate = d3_rate;
    pid_height.kp_rate = p4_rate;
    pid_height.ki_rate = i4_rate;
    pid_height.kd_rate = d4_rate;
  }

}



void POSEupdate(char *packet){

  char *ptr;
  uint8_t sum , ck , i;
  int16_t thr , aux1 , aux2 , aux3 , aux4 , aux5 , aux6;
  float y , r , p;

  sum = 0;
  ptr = packet;
  
  ptr = ptr + 1;
  thr = *(int16_t *)ptr;
  
  ptr = ptr + 2;
  y = *(int16_t *)ptr/100.0;
  
  ptr = ptr + 2;
  r = *(int16_t *)ptr/100.0;
  
  ptr = ptr + 2;
  p = *(int16_t *)ptr/100.0;
  
  ptr = ptr + 2;
  aux1 = *(int16_t *)ptr;
  
  ptr = ptr + 2;
  aux2 = *(int16_t *)ptr;
  
  ptr = ptr + 2;
  aux3 = *(int16_t *)ptr;
  
  ptr = ptr + 2;
  aux4 = *(int16_t *)ptr;
  
  ptr = ptr + 2;
  aux5 = *(int16_t *)ptr;
  
  ptr = ptr + 2;
  aux6 = *(int16_t *)ptr;

  ptr = ptr + 2;
  ck = *(char *)ptr;

  for(i = 1 ; i <= ptr - packet - 1 ; i++){
    sum += packet[i];
  }

  if(ck == sum){

    Qpost.throttle = (int)thr;
    Qpost.yaw = y;
    Qpost.roll = r;
    Qpost.pitch = p;

  }

}
