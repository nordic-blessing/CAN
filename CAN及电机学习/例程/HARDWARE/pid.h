#ifndef __PID_H
#define __PID_H
#include "stm32f4xx.h"


typedef struct
{
	float Setpoint;     //目标值
	double SunError;    //误差累计
	
	float P;
	float I;
	float D;
	
	float Last_error;
	float LLast_error;
	float Out_put;
}PIDTypeDef;


typedef struct{
	PIDTypeDef SpeedLoop;
	PIDTypeDef AngleLoop;
}PID_Struct_Motor_t;

extern PID_Struct_Motor_t S_Motor;

void SetPoint(PIDTypeDef *pid,float want);
void PID_Set_KP_KI_KD(PIDTypeDef *pid,float kp,float ki,float kd);
void PID_Init(PIDTypeDef *pid);
void PID_Set_Init(void);

int32_t  PID_PosLocCalc_Spd(PIDTypeDef *pid,int32_t cnt);
int32_t  PID_IncCalc(PIDTypeDef *pid, float Now_Point);
int32_t  PID_PosLocCalc(PIDTypeDef *pid, int32_t Now_Point);

#endif

