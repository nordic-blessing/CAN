#include "motor.h"




void Speed_Control(void)//�����ٶ��Լ����������ٶȺ������Ƕ��ٶ�
{

	get_moto_measure_2006(&motoS_chassis, canDat.M1_datBuf);//��ȡ��can���ص��ٶ�ֵ

	
	S_Motor.SpeedLoop.Out_put=PID_IncCalc(&S_Motor.SpeedLoop,motoS_chassis.speed_rpm);

		////can1����˳�� 1 2 3 4�ŵ��

	CAN1_Send_Msg(S_Motor.SpeedLoop.Out_put,S_Motor.SpeedLoop.Out_put,S_Motor.SpeedLoop.Out_put,S_Motor.SpeedLoop.Out_put);
	CAN2_Send_Msg(S_Motor.SpeedLoop.Out_put,S_Motor.SpeedLoop.Out_put,S_Motor.SpeedLoop.Out_put,S_Motor.SpeedLoop.Out_put);
}



void Restrain_Amp(u16 Amplitude,float input)
{
	if(input>Amplitude)input=Amplitude;
	if(input<-Amplitude)input=-Amplitude;
}

