#include "motor.h"




void Speed_Control(void)//期望速度以及期望方向速度和期望角度速度
{

	get_moto_measure_2006(&motoS_chassis, canDat.M1_datBuf);//获取由can返回的速度值

	
	S_Motor.SpeedLoop.Out_put=PID_IncCalc(&S_Motor.SpeedLoop,motoS_chassis.speed_rpm);

		////can1发送顺序 1 2 3 4号电机

	CAN1_Send_Msg(S_Motor.SpeedLoop.Out_put,S_Motor.SpeedLoop.Out_put,S_Motor.SpeedLoop.Out_put,S_Motor.SpeedLoop.Out_put);
	CAN2_Send_Msg(S_Motor.SpeedLoop.Out_put,S_Motor.SpeedLoop.Out_put,S_Motor.SpeedLoop.Out_put,S_Motor.SpeedLoop.Out_put);
}



void Restrain_Amp(u16 Amplitude,float input)
{
	if(input>Amplitude)input=Amplitude;
	if(input<-Amplitude)input=-Amplitude;
}

