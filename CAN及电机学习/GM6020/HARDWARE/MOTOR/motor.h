#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"
#include "can.h"


/*电机数据结构体*/
typedef struct
{
    uint16_t can_id;               //CAN id号
    int16_t  set_voltage;          //设置的电压值，控制速度
    uint16_t rotor_angle;          //当前角度
    int16_t  rotor_speed;          //当前速度
    int16_t  torque_current;       //当前电流
    uint8_t  temp;                 //电机温度
	int32_t  total_angle;          //电机转过总角度
	int16_t  total_cnt;            //电机转过总圈数 
	uint16_t  offset_angle;         //初始偏差
	uint16_t  last_angle;
	uint16_t  real_angle;
}moto_info_t;

//数据控制结构体
typedef struct
{
    uint16_t can_id;             //CAN id号
	int32_t angle_yaw;           //云台角
	int32_t angle_pitch;         //俯仰角
	uint8_t ball_take_flag;      //是否执行收球动作标志
	int16_t YAW_BIAS;            //云台自瞄偏差
}Control_type;

extern float SpeedWant[4];
extern float AngleWant[4];
extern moto_info_t motor_info[7];       //返回电机数据数组
extern Control_type control_info[2];    //控制结构体


uint8_t Speed_Control(moto_info_t motor_info[4], float V1,float V2,float V3,float V4);					//速度控制

uint8_t Angle_Control(moto_info_t motor_info[4], float A1,float A2,float A3,float A4);         //角度控制

int Motor_Set_Round_angle(moto_info_t motor_info[4], int16_t round0,int16_t angle0, int16_t round1,int16_t angle1);

void Motor_State_Init(void);  																		//电机状态初始化


#endif  /* __MOTOR_H */
