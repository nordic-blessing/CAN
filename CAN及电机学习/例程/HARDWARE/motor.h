#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"
#include "pid.h"
#include "can.h"



void Speed_Control(void);
void Restrain_Amp(u16 Amplitude,float speed);

#endif 

