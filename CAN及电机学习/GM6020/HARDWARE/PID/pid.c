#include "pid.h"
#include "math.h"

PIDTypeDef PID_SpeedLoop[4];    //电机速度环PID
PIDTypeDef PID_AngleLoop[4];    //电机角度环PID
PIDTypeDef PID_6020Loop1[4];    //云台PID
PIDTypeDef PID_6020Loop2[4];    //俯仰PID
PIDTypeDef PID_6020Loop3[4];    //俯仰PID
PIDTypeDef PID_3508Loop[4];    //收球PID
PIDTypeDef PID_PIXLoop[4];    //自瞄像素环

uint16_t Output_MAX = 16000;      //最大输出限制

void PID_Set_KP_KI_KD(PIDTypeDef *pid,float kp,float ki,float kd,u8 FLAG_integral_limit, float integral_limit, u8 FLAG_integral_separation,float integral_separation_limit)   //设置PID
{
  pid->P=kp;
  pid->I=ki;
  pid->D=kd;
  pid->FLAG_integral_separation=FLAG_integral_separation;
  pid->FLAG_integral_limit=FLAG_integral_limit;
  pid->integral_limit=integral_limit;
  pid->integral_separation_limit=integral_separation_limit;
}

void SetPoint(PIDTypeDef *pid,float Target)    //设置目标
{
	pid->Target=Target;
}

void PID_Init(PIDTypeDef *pid)   //PID结构体初始化
{
	 pid->Target=0.0;
	 pid->SumError=0.0;
	
	 pid->P=0.0;
	 pid->I=0.0;
	 pid->D=0.0;
	
	 pid->Last_error=0.0;
	 pid->L_Last_error=0.0;
	
	pid->FLAG_integral_separation=0;
	pid->FLAG_integral_limit=0;
	pid->integral_limit=0;
	pid->integral_separation_limit=0;
}


int32_t  PID_PosLocCalc(PIDTypeDef *pid, int32_t Now_Point)//位置式
{
    float Now_Error,d_Error;
	Now_Error=pid->Target-Now_Point;
		if(pid->FLAG_integral_separation==1)															//开启了积分分离
	{
		if(fabs(Now_Error) >= pid->integral_separation_limit)				//当偏差超过限度时，不进行积分累加
		{
			//do nothing
		}
		else
		{
			if(pid->FLAG_integral_limit==1)															//在开启了积分分离的前提下，判断是否积分限幅
				if((fabs(pid->SumError)>=pid->integral_limit)&&(pid->SumError*Now_Error>0))								//如果积分项超出范围，不进行积分累加
				{
					//do nothing
				}
				else
				{
					pid->SumError += Now_Error;																//偏差未超过限度，积分项未超过积分限幅，积分项正常累加
					if(fabs(pid->SumError)>pid->integral_limit)								//在此判断积分项累加后是否超过限度，如果超过,则限制在幅度值
					{
						if(pid->SumError<0)
							pid->SumError=(-1)*pid->integral_limit;               //负值
						if(pid->SumError>0)
							pid->SumError=pid->integral_limit;										//正值
					}
				}
			else
				pid->SumError += Now_Error;         													//偏差未超过限度，未开启积分限幅，积分项正常累加
		}
	}
	else																														//未开启积分分离
	{
		if(pid->FLAG_integral_limit==1)																//在未开启了积分分离的前提下，判断是否积分限幅
		{
			if((fabs(pid->SumError)>=pid->integral_limit)&&(pid->SumError*Now_Error>0))										//如果积分项超出范围，不进行积分累加
			{
					//do nothing
			}
			else                                                        //没有超过积分限幅
			{
				pid->SumError += Now_Error;																//关闭了积分分离，积分项未超过积分限幅，积分项正常累加
				if(fabs(pid->SumError)>pid->integral_limit)								//在此判断积分项累加后是否超过限度，如果超过,则限制在幅度值
				{
					if(pid->SumError<0)
						pid->SumError=(-1)*pid->integral_limit;               //负值
					if(pid->SumError>0)
						pid->SumError=pid->integral_limit;										//正值
				}
			}
		}
		else                                                          //积分限幅积分分离均未开启,直接加
		{
			pid->SumError += Now_Error;
		}
	}

	d_Error=Now_Error-pid->Last_error;
	
    pid->Last_error=Now_Error;
	
	pid->Output+= (int32_t)(pid->P*Now_Error+
		                    pid->I*pid->SumError+
	                        pid->D*d_Error);
	if(pid->Output>Output_MAX)
		pid->Output=Output_MAX;
	else if(pid->Output<-Output_MAX)
		pid->Output=-Output_MAX;
	return pid->Output;
}


int32_t PID_IncCalc(PIDTypeDef *pid,float Now_Point)//增量式
{
  float p_Error,Now_Error,d_Error,i_Error;
	
	Now_Error=pid->Target-Now_Point;
	
	p_Error=Now_Error-pid->Last_error;//p分量
	
	i_Error=Now_Error;//I分量
	
	d_Error=Now_Error-2*pid->Last_error+pid->L_Last_error;//D分量
	
    pid->Last_error=Now_Error;
	pid->L_Last_error=pid->Last_error;
	pid->Output+=(int32_t)(pid->P*p_Error+
		                   pid->I*i_Error+
	                       pid->D*d_Error);
	if(pid->Output>Output_MAX)
		pid->Output=Output_MAX;
	else if(pid->Output<-Output_MAX)
		pid->Output=-Output_MAX;
	return pid->Output;
}

void PID_Set_Init(void)                     //PID初始化赋值
{
	int i;
	for(i=0;i<4;i++)
	{		PID_Init(&PID_SpeedLoop[i]);
		PID_Init(&PID_AngleLoop[i]);	
		PID_Init(&PID_6020Loop1[i]);
		PID_Init(&PID_6020Loop2[i]);
		PID_Init(&PID_3508Loop[i]);
		PID_Init(&PID_PIXLoop[i]);
		//3508收球机构双环
		PID_Set_KP_KI_KD(&PID_SpeedLoop[i],7.5,0.005,0,0,0,0,0);  //30   3.5   0   //20   5   0//
		PID_Set_KP_KI_KD(&PID_3508Loop[i],0.1,0.01,0,0,0,0,0); //Inc1.82   0.07   3  /inc 10  20  5  两组数据较好   /  50  7.5   5/      50     15    0   /    50  25  0/0.16  0.02

		//6020角度环测试
		PID_Set_KP_KI_KD(&PID_AngleLoop[i],20,10,0,0,0,0,0); //Inc1.82   0.07   3  /inc 10  20  5  两组数据较好
		
		//云台
		PID_Set_KP_KI_KD(&PID_6020Loop1[i],40,0.9,0,0,4000,0,5000); //INC 50  10  0  /40    2.7    0 /   40  0.95  0
		
		//俯仰
		PID_Set_KP_KI_KD(&PID_6020Loop3[i],10,0.07,0,1,6000,0,1000); //60  2.7   0  /  10   0.3   0
		PID_Set_KP_KI_KD(&PID_6020Loop2[i],10,0.07,0,1,6000,0,1000); //60  2.7   0  /   10   0.3  0
		
		//自瞄像素环
		PID_Set_KP_KI_KD(&PID_PIXLoop[i],0.19,0,0,0,4000,0,1000);
	}
}
