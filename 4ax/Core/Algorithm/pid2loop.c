#include "pid.h"
#include "post.h"
#include "math.h"
#include "motor.h"

#define kp_rate_roll 0.0
#define kp_rate_pitch 0.0
#define kp_rate_yaw 0.0
#define kp_rate_height 0.0

#define ki_rate_roll 0.0
#define ki_rate_pitch 0.0
#define ki_rate_yaw 0.0
#define ki_rate_height 0.0

#define kd_rate_roll 0
#define kd_rate_pitch 0
#define kd_rate_yaw 0.0
#define kd_rate_height 0.0

#define kp_roll 0.0
#define kp_pitch 0.0
#define kp_yaw 0.0
#define kp_height 0.0

#define ki_roll 0.0
#define ki_pitch 0.0
#define ki_yaw 0.0
#define ki_height 0.0

#define kd_roll 0.0
#define kd_pitch 0.0
#define kd_yaw 0.0
#define kd_height 0.0


//積分限幅
#define pid_imax 500.0
#define pid_imin -500.0

extern post Qpost;
extern float roll , pitch , yaw;
extern short gyro[3];
extern int throttle;
extern uint8_t rx_Buffer[1024];

float height;

pst pid_roll = {

    .kp_rate = kp_rate_roll,
    .ki_rate = ki_rate_roll,
    .kd_rate = kd_rate_roll,
    .last_angv = 0,
    .kp = kp_roll,
    .ki = ki_roll,
    .kd = kd_roll,
    .last_ang = 0,
    .imax = pid_imax,
    .imin = pid_imin,
    .out = 0,
    .last_out = 0,
    .feedback = &roll,
    .ii = 0
};

pst pid_pitch = {

    .kp_rate = kp_rate_pitch,
    .ki_rate = ki_rate_pitch,
    .kd_rate = kd_rate_pitch,
    .last_angv = 0,
    .kp = kp_pitch,
    .ki = ki_pitch,
    .kd = kd_pitch,
    .last_ang = 0,
    .imax = pid_imax,
    .imin = pid_imin,
    .out = 0,
    .last_out = 0,
    .last_error = 0,
    .feedback = &pitch,
    .ii = 0
};

pst pid_yaw = {

    .kp_rate = kp_rate_yaw,
    .ki_rate = ki_rate_yaw,
    .kd_rate = kd_rate_yaw,
    .last_angv = 0,
    .kp = kp_yaw,
    .ki = ki_yaw,
    .kd = kd_yaw,
    .last_ang = 0,
    .imax = pid_imax,
    .imin = pid_imin,
    .out = 0,
    .last_out = 0,
    .feedback = &yaw,
    .ii = 0
};


pst pid_height = {

    .kp_rate = kp_rate_height,
    .ki_rate = ki_rate_height,
    .kd_rate = kd_rate_height,
    .last_angv = 0,
    .kp = kp_height,
    .ki = ki_height,
    .kd = kd_height,
    .last_ang = 0,
    .imax = pid_imax,
    .imin = pid_imin,
    .out = 0,
    .last_out = 0,
    .feedback = &height,
    .ii = 0
};

void AngPIDController(void *tskfreq){

    unsigned int *freq = (unsigned int *)tskfreq;

    while(1){

    int pid_pitch_value = limit(pid_control(Qpost.roll , Qpost.pitch , Qpost.yaw , 'P' , 0 ) , 200 , -200 );
    int pid_roll_value = limit(pid_control(Qpost.roll , Qpost.pitch , Qpost.yaw , 'R' , 0 ) , 200 , -200 );
    int pid_yaw_value = limit(pid_control(Qpost.roll , Qpost.pitch , Qpost.yaw , 'Y' , 0 ) , 200 , -200 );

    throttle = Qpost.throttle;

    //大於 60 度時停止馬達。
    if(fabs(pitch) > 50 || fabs(roll) > 50){throttle = 800;}

    if(throttle > 810){
        
        // ch1 = throttle - pid_pitch_value  + pid_yaw_value;
        // ch2 = throttle - pid_roll_value - pid_yaw_value;
        // ch3 = throttle + pid_pitch_value + pid_yaw_value;
        // ch4 = throttle + pid_roll_value - pid_yaw_value;
        ch1 = throttle - pid_pitch_value;
        ch2 = throttle + pid_pitch_value;
        ch3 = throttle + pid_pitch_value;
        ch4 = throttle - pid_pitch_value;

    }else{

        ch1 = throttle;
        ch2 = throttle;
        ch3 = throttle;
        ch4 = throttle;

    }

    vTaskDelay(*freq);
    
    }
}


int pid_control(float set_roll , float set_pitch , float set_yaw , char op , float h){

    float angv;

    if(op == 'P'){

        pid_pitch.error = *pid_pitch.feedback - set_pitch;

        if(fabs(pid_pitch.error) > 0.01)
        {
            angv = (float)gyro[1] * 0.0305175;

            // PID 外環
            pid_pitch.p = pid_pitch.error;    
            pid_pitch.i += pid_pitch.error;
            pid_pitch.i = limit(pid_pitch.i , pid_pitch.imax , pid_pitch.imin);
            pid_pitch.d = angv;
            pid_pitch.last_ang = *pid_pitch.feedback;
            pid_pitch.out = pid_pitch.kp * pid_pitch.p + pid_pitch.ki * pid_pitch.i + pid_pitch.kd * pid_pitch.d;
            
            // PID 內環
            pid_pitch.p = pid_pitch.out + gyro[1];
            pid_pitch.ii += pid_pitch.p;
            pid_pitch.ii = limit(pid_pitch.ii , pid_pitch.imax , pid_pitch.imin);
            pid_pitch.d = angv - pid_pitch.last_angv;
            pid_pitch.out = pid_pitch.kp_rate * pid_pitch.p + pid_pitch.ki_rate * pid_pitch.ii + pid_pitch.kd_rate * pid_pitch.d;
            pid_pitch.last_angv = angv;

            return (int)pid_pitch.out;
        }

        return (int)pid_pitch.last_out;

    }else if(op == 'R'){

        pid_roll.error = *pid_roll.feedback - set_roll;

        if(fabs(pid_roll.error) > 0.01)
        {
            angv = (float)gyro[0] * 0.0305175;

            // PID 外環
            pid_roll.p = pid_roll.error;    
            pid_roll.i += pid_roll.error;
            pid_roll.i = limit(pid_roll.i , pid_roll.imax , pid_roll.imin);
            pid_roll.d = angv;
            pid_roll.last_ang = *pid_roll.feedback;
            pid_roll.out = pid_roll.kp * pid_roll.p + pid_roll.ki * pid_roll.i + pid_roll.kd * pid_roll.d;

            // PID 內環
            pid_roll.p = pid_roll.out + gyro[0];
            pid_roll.ii += pid_roll.p;
            pid_roll.ii = limit(pid_roll.ii , pid_roll.imax , pid_roll.imin);
            pid_roll.d = angv - pid_roll.last_angv;
            pid_roll.out = pid_roll.kp_rate * pid_roll.p + pid_roll.ki_rate * pid_roll.ii + pid_roll.kd_rate * pid_roll.d;
            pid_roll.last_angv = angv;

            return (int)pid_roll.out;
        }

        return (int)pid_roll.last_out;

    }else if(op == 'Y'){

        pid_yaw.error = *pid_yaw.feedback - set_pitch;

        if(fabs(pid_yaw.error) > 0.01)
        {

            pid_yaw.p = pid_yaw.error;    
            pid_yaw.i += pid_yaw.error;
            pid_yaw.i = limit(pid_yaw.i , pid_yaw.imax , pid_roll.imin);
            pid_yaw.d = (float)gyro[2] * 0.0305175;
            pid_yaw.out = pid_yaw.kp * pid_yaw.p + pid_yaw.ki * pid_yaw.i + pid_yaw.kd * pid_yaw.d;
            pid_yaw.last_ang = *pid_yaw.feedback;
            pid_yaw.last_error = pid_yaw.error;  
            pid_yaw.last_out = pid_yaw.out;
            
            return (int)pid_yaw.out;

        }

        return (int)pid_yaw.last_out;
    
    }else if(op == 'H'){

        // 高度 PID : 外環控高度 , 內環控速度
        static float vnow , vpre , hpre , lastp;
        float dt = 0.002;

        vnow = limit((*pid_height.feedback - hpre)/dt , -300 , 300);
        vnow = vnow * 0.2 + vpre * 0.8;
        vpre = vnow;
        hpre = *pid_height.feedback;

        // PID 外環
        pid_height.out = pid_height.kp * (h - *pid_height.feedback);

        // PID 內環
        pid_height.p = pid_height.out - vnow;
        pid_height.i += pid_height.p;
        pid_height.d = pid_height.p - lastp;
        pid_height.out = pid_height.kp_rate * pid_height.p + pid_height.ki_rate * pid_height.i + pid_height.kd_rate * pid_height.d;
        lastp = pid_height.p;

    }
    
}


float limit(float n , float max , float min){
    
    if(n > max){
        n = max;
    }else if(n < min){
        n = min;
    }

    return n;
}

