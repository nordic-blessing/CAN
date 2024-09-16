#include "motor.h"
#include "sys.h"
#include "usart.h"
#include "can.h"

float SpeedWant[4];
float AngleWant[4];
float TotalWant[4];

void Motor_State_Init(void)
{
	int i = 0;
	for(i = 0;i < 4;i++)
	{
		motor_info[i].can_id = 0; 
		motor_info[i].rotor_angle = 0;
		motor_info[i].rotor_speed = 0;
		motor_info[i].set_voltage = 0;
		motor_info[i].temp = 0;
		motor_info[i].torque_current = 0;
		motor_info[i].total_angle = 0;
		motor_info[i].total_cnt = 0;   //电机转过总圈数
		motor_info[i].offset_angle = 0;//初始偏差
		motor_info[i].last_angle = 0;
	}
}
//主控外部控制结构体初始化
void Control_Info_Init(void)
{
	int i = 0;
	for(i = 0;i < 2;i++)
	{
		control_info[i].can_id = 0;
		control_info[i].angle_yaw = 0;
		control_info[i].angle_pitch = 0;
		control_info[i].YAW_BIAS = 0;
		control_info[i].ball_take_flag = 0;
	}
}
//3508取球控制
uint8_t Speed_Control(moto_info_t motor_info[4], float V1,float V2,float V3,float V4)
{
	 int i = 0;
	 int can_send_flag = 0;
	 int16_t Speed_Output[4];

     SetPoint(&PID_SpeedLoop[0],V1);	//写入期望值
	 SetPoint(&PID_SpeedLoop[1],V2);
	 SetPoint(&PID_SpeedLoop[2],V3);
	 SetPoint(&PID_SpeedLoop[3],V4);
	
	 Speed_Output[0] = PID_IncCalc(&PID_SpeedLoop[0],motor_info[2].rotor_speed);  //pid调节
		 
	 if(Speed_Output[i]>16000)		//限幅
	 Speed_Output[i]= 16000;
	 else if(Speed_Output[i]<-16000)
	 Speed_Output[i]=-16000;
	 
     can_send_flag = set_motor_voltage(1, Speed_Output[0],0,0,0);
	 return can_send_flag;
}

uint8_t Angle_Control(moto_info_t motor_info[4], float A1,float A2,float A3,float A4)
{
	int can_send_flag = 1;
	int i = 0;
	int16_t Angle_Output[4];
	
	SetPoint(&PID_AngleLoop[0],A1);        //赋给角度环期望值
	SetPoint(&PID_AngleLoop[1],A2);        
	SetPoint(&PID_AngleLoop[2],A3);        
	SetPoint(&PID_AngleLoop[3],A4);        
	
	for(i=0;i<4;i++)
	{
		Angle_Output[i]=PID_IncCalc(&PID_AngleLoop[i],motor_info[i].rotor_angle);//计算输出
		if(Angle_Output[i]>30000)		  //限幅
		Angle_Output[i]=30000;
		else if(Angle_Output[i]<-30000)
		Angle_Output[i]=-30000;
		motor_info[i].set_voltage = Angle_Output[i];
	}
	can_send_flag = set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage, motor_info[2].set_voltage, motor_info[3].set_voltage);		     //CAN发送数据
	return can_send_flag;
}

int Motor_Set_Round_angle(moto_info_t motor_info[4], int16_t round0,int16_t angle0, int16_t round1,int16_t angle1)
{
	int angle_total_0 = 0;
	int angle_total_1 = 0;
	int angle_total_out[4] = {0};
	int can_send_flag = 1;
	int i = 0;
	
	angle_total_0 = round0 * 8192 + angle0;
	angle_total_1 = round1 * 8192 + angle1;
	
	SetPoint(&PID_6020Loop1[0],angle_total_0);	//写入期望值
	SetPoint(&PID_6020Loop2[1],angle_total_1);
	SetPoint(&PID_6020Loop3[1],angle_total_1);
	
	angle_total_out[0] = PID_IncCalc(&PID_6020Loop1[0],motor_info[0].total_angle);  //pid调节
	if(motor_info[0].total_angle < angle_total_1)
	{
		angle_total_out[1] = PID_IncCalc(&PID_6020Loop2[1],motor_info[1].total_angle);  //pid调节
	}
	else if(motor_info[0].total_angle >= angle_total_1)
	{
		angle_total_out[1] = PID_IncCalc(&PID_6020Loop3[1],motor_info[1].total_angle);  //pid调节
	}
	for(i = 0;i<4;i++)
	{
		 if(angle_total_out[i]>15000)//限幅
		 angle_total_out[i]=15000;
		 else if(angle_total_out[i]<-15000)
		 angle_total_out[i]=-15000;
	}
	can_send_flag = set_motor_voltage(0, angle_total_out[0], angle_total_out[1], angle_total_out[2], angle_total_out[3]);//CAN发送数据
	return can_send_flag;
}
