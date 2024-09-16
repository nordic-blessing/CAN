#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"
#include "can.h"


/*������ݽṹ��*/
typedef struct
{
    uint16_t can_id;               //CAN id��
    int16_t  set_voltage;          //���õĵ�ѹֵ�������ٶ�
    uint16_t rotor_angle;          //��ǰ�Ƕ�
    int16_t  rotor_speed;          //��ǰ�ٶ�
    int16_t  torque_current;       //��ǰ����
    uint8_t  temp;                 //����¶�
	int32_t  total_angle;          //���ת���ܽǶ�
	int16_t  total_cnt;            //���ת����Ȧ�� 
	uint16_t  offset_angle;         //��ʼƫ��
	uint16_t  last_angle;
	uint16_t  real_angle;
}moto_info_t;

//���ݿ��ƽṹ��
typedef struct
{
    uint16_t can_id;             //CAN id��
	int32_t angle_yaw;           //��̨��
	int32_t angle_pitch;         //������
	uint8_t ball_take_flag;      //�Ƿ�ִ����������־
	int16_t YAW_BIAS;            //��̨����ƫ��
}Control_type;

extern float SpeedWant[4];
extern float AngleWant[4];
extern moto_info_t motor_info[7];       //���ص����������
extern Control_type control_info[2];    //���ƽṹ��


uint8_t Speed_Control(moto_info_t motor_info[4], float V1,float V2,float V3,float V4);					//�ٶȿ���

uint8_t Angle_Control(moto_info_t motor_info[4], float A1,float A2,float A3,float A4);         //�Ƕȿ���

int Motor_Set_Round_angle(moto_info_t motor_info[4], int16_t round0,int16_t angle0, int16_t round1,int16_t angle1);

void Motor_State_Init(void);  																		//���״̬��ʼ��


#endif  /* __MOTOR_H */
