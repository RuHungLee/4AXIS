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
extern pst pid_roll , pid_pitch , pid_yaw;
extern int throttle;
extern bmp_t bmp;
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
    bmp_init(&bmp);

  	while(1){

      // Read_DMP();
      // ToEulerAngles();
      // // printf("short : %d , short : %d , short : %d\n" , (int)gyro[0] , (int)gyro[1] , (int)gyro[2]);
      // pitch = pitch + 2;
      // roll = roll;
      // if(cnt % 20 == 0){
      //   ANO_DT_Send_Status(roll , pitch , yaw , 0 , 0 , 0);
      // }
      // packetHandler();
      // motor_update();  
      // cnt = cnt + 1;
      
      bmp.uncomp.temp = get_ut();
      bmp.data.temp = get_temp(&bmp);
      bmp.uncomp.press = get_up(bmp.oss);
      bmp.data.press = get_pressure(bmp);
      bmp.data.altitude = get_altitude(&bmp);

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
  float p1_rate , i1_rate , d1_rate , p2_rate , i2_rate , d2_rate , p3_rate , i3_rate , d3_rate;

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

    pid_roll.kp_rate = p1_rate;
    pid_roll.ki_rate = i1_rate;
    pid_roll.kd_rate = d1_rate;
    pid_pitch.kp_rate = p2_rate;
    pid_pitch.ki_rate = i2_rate;
    pid_pitch.kd_rate = d2_rate;
    pid_yaw.kp_rate = p3_rate;
    pid_yaw.ki_rate = i3_rate;
    pid_yaw.kd_rate = d3_rate;

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
