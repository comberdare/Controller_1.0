#include <math.h>
#include "pid.h"


void PID_init(pid_config *pid){
    pid->SetValue = 2.8;		//##
    pid->ActualValue = 0.0;
    pid->err = 0.0;
    pid->err_last = 0.0;
    pid->OutValue = 0.0;
    pid->integral = 0.0;
    pid->Kp = 0.5;
    pid->Ki = 0.1;         
    pid->Kd = 0;
    pid->umax = 100;
    pid->umin = -100;
	pid->err_threshold = 100; 
}

float PID_realize(pid_config *pid){
	int index;
    pid->err = pid->SetValue - pid->ActualValue;

	if(pid->ActualValue > pid->umax)       
	{
		if(fabs(pid->err) > pid->err_threshold)      
		{
			index = 0;
        }
		else
		{
            index = 1;
            if(pid->err < 0)
            {
              pid->integral += pid->err;
            }
        }
    }
	else if(pid->ActualValue < pid->umin)
	{
		if(fabs(pid->err) > pid->err_threshold)      
        {
			index = 0;
        }
		else
		{
            index = 1;
            if(pid->err>0)
            {
            pid->integral += pid->err;
            }
        }
    }
	else
	{
        if(fabs(pid->err) > 2*pid->err_threshold)                    
        {
            index=0;
        }
		else
		{
            index=1;
            pid->integral += pid->err;
        }
    }

    pid->OutValue = pid->Kp * pid->err + index * pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);
    pid->err_last = pid->err;
    return pid->OutValue;
}
