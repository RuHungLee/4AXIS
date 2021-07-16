int pid_control(float set_roll , float set_pitch , float set_yaw , char op){


    if(op == 'P'){

        pid_pitch.error = *pid_pitch.feedback - set_pitch;

        if(fabs(pid_pitch.error) > 0.01)
        {
            // PID 外環
            pid_pitch.p = pid_pitch.error;    
            pid_pitch.i += pid_pitch.error;
            pid_pitch.i = limit(pid_pitch.i , pid_pitch.imax , pid_pitch.imin);
            pid_pitch.d = pid_pitch.feedback - pid_pitch.last_ang;
            pid_pitch.last_ang = pid_pitch.feedback;
            pid_pitch.out = pid_pitch.kp * pid_pitch.p + pid_pitch.ki * pid_pitch.i + pid_pitch.kd * pid_pitch.d;
    
            // PID 內環
            pid_pitch.p = pid_pitch.out + gyro[1];
            pid_pitch.d = gyro[1] - pid_pitch.last_angv;
            pid_pitch.out = pid_pitch.kp_rate * pid_pitch.p + pid_pitch.kd_rate * pid_pitch.d;
            pid_pitch.last_angv = gyro[1];

            return (int)pid_pitch.out;
        }

        return (int)pid_pitch.last_out;

    }else if(op == 'R'){

        pid_roll.error = *pid_roll.feedback - set_pitch;

        if(fabs(pid_roll.error) > 0.01)
        {
            // PID 外環
            pid_roll.p = pid_roll.error;    
            pid_roll.i += pid_roll.error;
            pid_roll.i = limit(pid_roll.i , pid_roll.imax , pid_roll.imin);
            pid_roll.d = pid_roll.feedback - pid_roll.last_ang;
            pid_roll.last_ang = pid_roll.feedback;
            pid_roll.out = pid_roll.kp * pid_roll.p + pid_roll.ki * pid_roll.i + pid_roll.kd * pid_roll.d;
    
            // PID 內環
            pid_roll.p = pid_roll.out + gyro[0];
            pid_roll.d = gyro[0] - pid_roll.last_angv;
            pid_roll.out = pid_roll.kp_rate * pid_roll.p + pid_roll.kd_rate * pid_roll.d;
            pid_roll.last_angv = gyro[0];

            return (int)pid_roll.out;
        }

        return (int)pid_roll.last_out;

    }else if(op == 'Y'){

        pid_yaw.error = *pid_yaw.feedback - set_pitch;

        if(fabs(pid_yaw.error) > 0.01)
        {
            // PID 外環
            pid_yaw.p = pid_yaw.error;    
            pid_yaw.i += pid_yaw.error;
            pid_yaw.i = limit(pid_yaw.i , pid_yaw.imax , pid_yaw.imin);
            pid_yaw.d = pid_yaw.feedback - pid_yaw.last_ang;
            pid_yaw.last_ang = pid_yaw.feedback;
            pid_yaw.out = pid_yaw.kp * pid_yaw.p + pid_yaw.ki * pid_yaw.i + pid_yaw.kd * pid_yaw.d;
    
            // PID 內環
            pid_yaw.p = pid_yaw.out + gyro[2];
            pid_yaw.d = gyro[2] - pid_yaw.last_angv;
            pid_yaw.out = pid_yaw.kp_rate * pid_yaw.p + pid_yaw.kd_rate * pid_yaw.d;
            pid_yaw.last_angv = gyro[2];

            return (int)pid_yaw.out;
        }

        return (int)pid_yaw.last_out;
    
}
