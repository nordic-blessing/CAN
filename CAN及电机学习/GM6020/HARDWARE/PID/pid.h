#ifndef __PID_H_
#define __PID_H_

#include "stm32f4xx.h"
typedef struct
{
	float Target;
	double SumError;
	
	float P;
	float I;
	float D;
	
	float Last_error;
	float L_Last_error;
	float Output;
	
	float FLAG_integral_separation;
	float FLAG_integral_limit;
	float integral_limit;
	float integral_separation_limit;
}PIDTypeDef;

extern PIDTypeDef PID_SpeedLoop[4];

extern PIDTypeDef PID_AngleLoop[4];

extern PIDTypeDef PID_6020Loop1[4];
extern PIDTypeDef PID_6020Loop2[4];
extern PIDTypeDef PID_6020Loop3[4];

extern PIDTypeDef PID_3508Loop[4];

extern PIDTypeDef PID_PIXLoop[4];


void SetPoint(PIDTypeDef *pid,float target);
//void PID_Set_KP_KI_KD(PIDTypeDef *pid,float kp,float ki,float kd);
void PID_Set_KP_KI_KD(PIDTypeDef *pid,float kp,float ki,float kd,uint8_t FLAG_integral_limit, float integral_limit, uint8_t FLAG_integral_separation,float integral_separation_limit);
void PID_Init(PIDTypeDef *pid);
void PID_Set_Init(void);
float PID_realise(PIDTypeDef *PID, float current );

int32_t  PID_IncCalc(PIDTypeDef *pid, float Now_Point);
int32_t  PID_PosLocCalc(PIDTypeDef *pid, int32_t Now_Point);

#endif

