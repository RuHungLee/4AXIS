#ifndef _DATA_TRANSFER_H
#define _DATA_TRANSFER_H

#include "stm32f4xx.h"

#define u8 uint8_t
#define u16 uint16_t
#define s16 short
#define vs16 short
#define s32 int
#define vs32 int

typedef struct 
{
        u8 send_version;
        u8 send_status;
        u8 send_senser;
        u8 send_pid1;
        u8 send_pid2;
        u8 send_pid3;
        u8 send_pid4;
        u8 send_pid5;
        u8 send_pid6;
        u8 send_rcdata;
        u8 send_offset;
        u8 send_motopwm;
        u8 send_power;

}dt_flag_t;

extern dt_flag_t f;

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_Check(u8 head, u8 check_sum);


#endif
