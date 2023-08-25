typedef struct{
    float SetValue;            //定义设定值
    float ActualValue;         //定义实际值
    float err;                 //定义偏差值
    float err_last;            //定义上一个偏差值
    float Kp, Ki, Kd;          //定义比例、积分、微分系数
    float OutValue;            //定义控制执行器的变量
    float integral;            //定义积分值
    float umax;
    float umin;
	float err_threshold;
}pid_config;

void PID_init(pid_config *pid);
float PID_realize(pid_config *pid);
