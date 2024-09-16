#include "pid.h"
#include "math.h"

PID_Struct_Motor_t S_Motor;



void PID_Set_KP_KI_KD(PIDTypeDef *pid,float kp,float ki,float kd)
{
  pid->P=kp;
  pid->I=ki;
  pid->D=kd;
}


void SetPoint(PIDTypeDef *pid,float want)
{
	pid->Setpoint=want;
}


void PID_Init(PIDTypeDef *pid)
{
	 pid->Setpoint=0.0;
	 pid->SunError=0.0;
	
	 pid->P=0.0;
	 pid->I=0.0;
	 pid->D=0.0;
	
	 pid->Last_error=0.0;
	 pid->LLast_error=0.0;
}


int32_t  PID_PosLocCalc(PIDTypeDef *pid, int32_t Now_Point)//λ��ʽ
{
  float Now_Error,d_Error;
	Now_Error=pid->Setpoint-Now_Point;
	pid->SunError+=Now_Error;//�����޷�
  if(pid->SunError>4000)
		pid->SunError=4000;
	else if(pid->SunError<-4000)
		pid->SunError=-4000;
	d_Error=Now_Error-pid->Last_error;
  pid->Last_error=Now_Error;
	return (int32_t)(pid->P*Now_Error+
		               pid->I*pid->SunError+
	                 pid->D*d_Error);
}


int32_t PID_IncCalc(PIDTypeDef *pid,float Now_Point)//����ʽ
{
  float p_Error,Now_Error,d_Error,i_Error;
	Now_Error=pid->Setpoint-Now_Point;
	p_Error=Now_Error-pid->Last_error;//p����
	i_Error=Now_Error;//I����
	d_Error=Now_Error-2*pid->Last_error+pid->LLast_error;//D����
  pid->Last_error=Now_Error;
	pid->LLast_error=pid->Last_error;
	pid->Out_put+=(int32_t)(pid->P*p_Error+
		             pid->I*i_Error+
	               pid->D*d_Error);
	if(pid->Out_put>16384)
		pid->Out_put=16384;
	else if (pid->Out_put<-16384)
		pid->Out_put=-16384;
	return pid->Out_put;
}



