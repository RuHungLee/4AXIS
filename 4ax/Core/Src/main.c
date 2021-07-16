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

#define CCM_RAM __attribute__((section(".ccmram")))

extern float roll , pitch , yaw;
extern pst pid_roll , pid_pitch , pid_yaw;
extern int throttle;
extern uint8_t rx_Buffer[1024];
extern uint8_t packetRecved;


post Qpost = {

    .throttle = 0,
    .roll = 0,
    .pitch = 0,
    .yaw = 0

};

void packetHandler(void);
void PIDupdate(char *);
void POSEupdate(char *);

int main(void)
{

   	//驅動以及網路初始化
   	sysInit();
  	MPU6050_initialize();
  	DMP_Init();
  	ESP_Init();
      	
    int cnt = 0;

  	while(1){

  		Read_DMP();
  		ToEulerAngles();
      // if(cnt % 10 == 0){
  		  // ANO_DT_Send_Status(roll, pitch , yaw , 0 , 0 , 0);
      // }
  		packetHandler();
  		motor_update();  
      cnt = cnt + 1;
      // printf("cnt : %d\n" , cnt++);
      // printf("roll : %.2f\n" , Qpost.roll);
      // printf("pitch : %.2f\n" , Qpost.pitch);
      // printf("yaw : %.2f\n" , Qpost.yaw);
      // printf("kp : %.2f\n" , pid_pitch.kp);
      // printf("ki : %.2f\n" , pid_pitch.ki);
      // printf("kd : %.2f\n" , pid_pitch.kd);
    
    }
}
 

void packetHandler(void)
{

  if(packetRecved){

    char *ptr = strstr((char *)rx_Buffer , "\xaa\xaf");
    
    if(ptr == NULL){return;}
    
    ptr += 2;

    switch(ptr[0]){
      
      case '\x10':

        PIDupdate(ptr+1);

      case '\x03':

        POSEupdate(ptr+1);
    }
  }

  packetRecved = 0;

}


void PIDupdate(char *packet){

  char *ptr;
  uint8_t sum , ck , i;
  float p1 , i1 , d1 , p2 , i2 , d2 , p3 , i3 , d3;

  sum = 0;
  ptr = packet;
  
  
  ptr = ptr + 1;
  p1 = *(uint16_t *)ptr/100.0;
  
  ptr = ptr + 2;
  i1 = *(uint16_t *)ptr/100.0;
  
  ptr = ptr + 2;
  d1 = *(uint16_t *)ptr/100.0;
  
  ptr = ptr + 2;
  p2 = *(uint16_t *)ptr/100.0;
  
  ptr = ptr + 2;
  i2 = *(uint16_t *)ptr/100.0;
  
  ptr = ptr + 2;
  d2 = *(uint16_t *)ptr/100.0;
  
  ptr = ptr + 2;
  p3 = *(uint16_t *)ptr/100.0;
  
  ptr = ptr + 2;
  i3 = *(uint16_t *)ptr/100.0;
  
  ptr = ptr + 2;
  d3 = *(uint16_t *)ptr/100.0;
  
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
