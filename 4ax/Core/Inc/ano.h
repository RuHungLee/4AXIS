// drone to pc

void Send_Sensor(short , short , short , short , short , short , short , short , short);

typedef struct droneSensor{

    int header;
    char funcID;
    char len;
    short acc_x;
    short acc_y;
    short acc_z;
    short gyro_x;
    short gyro_y;
    short gyro_z;
    short mag_x;
    short mag_y;
    short mag_z;
    char sum;
    char end;
    
}droneSensor;

// pc to drone
