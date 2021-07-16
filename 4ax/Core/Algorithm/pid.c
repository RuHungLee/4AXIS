#include "pid.h"
#include "post.h"
#include "math.h"
#include "motor.h"

#define kp_rate_roll 0.0
#define kp_rate_pitch 0.0
#define kp_rate_yaw 0.0

#define ki_rate_roll 0.0
#define ki_rate_pitch 0.0
#define ki_rate_yaw 0.0

#define kd_rate_roll 0.0
#define kd_rate_pitch 0.0
#define kd_rate_yaw 0.0

#define kp_roll 2.0
#define kp_pitch 1.8
#define kp_yaw 1.0

#define ki_roll 0.0
#define ki_pitch 0.005
#define ki_yaw 0.0

#define kd_roll 1.75
#define kd_pitch 1.45
#define kd_yaw 0.0

//積分限幅
#define pid_imax 100.0
#define pid_imin -100.0

extern post Qpost;
extern float roll , pitch , yaw;
extern short gyro[3];
extern int throttle;
extern uint8_t rx_Buffer[1024];


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
    .feedback = &roll
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
    .feedback = &pitch
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
    .feedback = &yaw
};



void motor_update(){


    int pid_pitch_value = limit(pid_control(Qpost.roll , Qpost.pitch , Qpost.yaw , 'P') , 200 , -200 );
    int pid_roll_value = limit(pid_control(Qpost.roll , Qpost.pitch , Qpost.yaw , 'R') , 200 , -200 );
    int pid_yaw_value = limit(pid_control(Qpost.roll , Qpost.pitch , Qpost.yaw , 'Y') , 200 , -200 );

    throttle = Qpost.throttle;

    //大於 60 度時停止馬達。
    if(fabs(pitch) > 50 || fabs(roll) > 50){throttle = 800;}

    if(throttle > 850){
        
        ch1 = throttle - pid_pitch_value  + pid_yaw_value;
        ch2 = throttle - pid_roll_value + 15 - pid_yaw_value;
        ch3 = throttle + pid_pitch_value + 20 + pid_yaw_value; 
        ch4 = throttle + pid_roll_value - pid_yaw_value;


    }else{

        ch1 = throttle - 3;
        ch2 = throttle;
        ch3 = throttle;
        ch4 = throttle - 5;

    }

}


int pid_control(float set_roll , float set_pitch , float set_yaw , char op){

    // short pid_roll_bias , pid_pitch_bias   , pid_yaw_bias;

    // 將當前角度和目標角度誤差輸入 PID 控制器

    

    // PID 外環控制角度
    // pid_pitch.d = *pid_pitch.feedback - pid_pitch.last_ang;
    // pid_pitch.last_ang = *pid_pitch.feedback; 


    if(op == 'P'){

        pid_pitch.error = *pid_pitch.feedback - set_pitch;

        if(fabs(pid_pitch.error) > 1)
        {

            pid_pitch.p = pid_pitch.error;    
            pid_pitch.i += pid_pitch.error;
            pid_pitch.i = limit(pid_pitch.i , pid_pitch.imax , pid_pitch.imin);
            pid_pitch.d = (float)gyro[1] * 0.0305175;
            pid_pitch.out = pid_pitch.kp * pid_pitch.p + pid_pitch.ki * pid_pitch.i + pid_pitch.kd * pid_pitch.d;
            pid_pitch.last_ang = *pid_pitch.feedback;
            pid_pitch.last_error = pid_pitch.error;  

            pid_pitch.last_out = pid_pitch.out;
            
            return (int)pid_pitch.out;
        }

        return (int)pid_pitch.last_out;

    }else if(op == 'R'){

        pid_roll.error = *pid_roll.feedback - set_roll;

        if(fabs(pid_roll.error) > 1)
        {

            pid_roll.p = pid_roll.error;    
            pid_roll.i += pid_roll.error;
            pid_roll.i = limit(pid_roll.i , pid_roll.imax , pid_roll.imin);
            pid_roll.d = (float)gyro[0] * 0.0305175;
            pid_roll.out = pid_roll.kp * pid_roll.p + pid_roll.ki * pid_roll.i + pid_roll.kd * pid_roll.d;
            pid_roll.last_ang = *pid_roll.feedback;
            pid_roll.last_error = pid_roll.error;  

            pid_roll.last_out = pid_roll.out;
            
            return (int)pid_roll.out;
        }

        return (int)pid_roll.last_out;

    }else if(op == 'Y'){

        pid_yaw.error = *pid_yaw.feedback - set_roll;

        if(fabs(pid_yaw.error) > 1)
        {

            pid_yaw.p = pid_yaw.error;    
            pid_yaw.i += pid_yaw.error;
            pid_yaw.i = limit(pid_yaw.i , pid_yaw.imax , pid_roll.imin);
            pid_yaw.d = (float)gyro[0] * 0.0305175;
            pid_yaw.out = pid_yaw.kp * pid_yaw.p + pid_yaw.ki * pid_yaw.i + pid_yaw.kd * pid_yaw.d;
            pid_yaw.last_ang = *pid_yaw.feedback;
            pid_yaw.last_error = pid_yaw.error;  

            pid_yaw.last_out = pid_yaw.out;
            
            return (int)pid_yaw.out;
        }

        return (int)pid_yaw.last_out;

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



// pid_st pid_roll = { 
//     .InnerLast = 0;
//     .OutterLast = 0;
//     .Feedback = &row;
//     .i = 0;
//     .Channel1 = &ch1;
//     .Channel2 = &ch3;
//     .Gryo = &gryo[0];
// }

// pid_st pid_pitch = { 
//     .InnerLast = 0;
//     .OutterLast = 0;
//     .Feedback = &pitch;
//     .i = 0;
//     .Channel1 = &ch2;
//     .Channel2 = &ch4;
//     .Gryo = &gryo[1];

// }

// void pid_SingleAxis(pid_pst temp, float setPoint) {
//     temp->Error = *temp->Feedback - setPoint;
// //Outter Loop PID
//     temp->i += temp->Error;
//     if (temp->i > PID_IMAX) temp->i = PID_IMAX;
//     else if (temp->i < PID_IMIN) temp->i = PID_IMIN;

//     temp->d = *temp->Feedback - temp->OutterLast;

//     temp->output = (short)(OUTTER_LOOP_KP * (temp->Error) + OUTTER_LOOP_KI * temp->i + OUTTER_LOOP_KD * temp->d);
//     temp->OutterLast = *temp->Feedback; //Save Old Data
// //Inner Loop PD
//     temp->p = temp->output + *temp->Gyro;
//     temp->d = *temp->Gyro - temp->InnerLast;
//     temp->output = (short)(INNER_LOOP_KP * temp->p + INNER_LOOP_KD * temp->d);

//     if (ch1+temp->output > THROTTLE_MAX) *temp->Channel1 = THROTTLE_MAX;
//     else if (*temp->Channel1+temp->output < THROTTLE_MIN) *temp->Channel1 = THROTTLE_MIN;
//     else *temp->Channel1 += (short)temp->output;

//     if (*temp->Channel2-temp->output > THROTTLE_MAX) *temp->Channel2 = THROTTLE_MAX;
//     else if (*temp->Channel2-temp->output < THROTTLE_MIN) *temp->Channel2 = THROTTLE_MIN;
//     else *temp->Channel2 -= (short)temp->output;

//     temp->InnerLast = *temp->Gyro;
// }
