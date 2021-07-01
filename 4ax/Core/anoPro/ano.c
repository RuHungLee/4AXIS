#include "ano.h"
#include "string.h"
#include "stdio.h"

void Send_Sensor(short acc_x , short acc_y , short acc_z , short gyro_x , short gyro_y , \
    short gyro_z ,short mag_x , short mag_y , short mag_z){


    char *packet = NULL;
    char i , sum = 0;
    const char len = 24;

    droneSensor ds = {

        .header = 0x41414141,
        .funcID = 0x2,
        .len = 0x12,
        .acc_x = acc_x,
        .acc_y = acc_y,
        .acc_z = acc_z,
        .gyro_x = gyro_x,
        .gyro_y = gyro_y,
        .gyro_z = gyro_z,
        .mag_x = mag_x,
        .mag_y = mag_y,
        .mag_z = mag_z,
    };

    packet = (char *)&ds;
    
    for(i = 0 ; i < len ; i++){
        sum += packet[i];
    }

    packet[24] = sum;

    for(i = 0 ; i < len + 1 ; i++){
        putchar(packet[i]);
    }
}
