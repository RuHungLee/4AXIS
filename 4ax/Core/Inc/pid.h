#ifndef PID_H
#define PID_H

// extern float kp_rate 0.0
// extern float ki_rate 0.0
// extern float kd_rate 0.0
// extern float kp 0.0
// extern float ki 0.0
// extern float kd 0.0
float limit(float , float , float);
int pid_control(float , float , float);
void motor_update();

typedef struct pid_struct{
    
    float error , last_error;
    float kp;
    float ki;
    float kd;
    float last_angv; //inner
    float kp_rate;
    float ki_rate;
    float kd_rate;
    float last_ang; //outer
    float *feedback;
    float p , i , d , imax , imin;
    float out , last_out;

}pst;


#endif

