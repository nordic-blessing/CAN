#ifndef __CAN_H
#define __CAN_H	 
#include "stm32f4xx.h"   
#include "led.h"

typedef struct
{
	u8 M1_datBuf[8];
	u8 M2_datBuf[8];
	u8 M3_datBuf[8];
	u8 M4_datBuf[8];
	u8 M5_datBuf[8];
	u8 M6_datBuf[8];
	u8 M7_datBuf[8];
	u8 M8_datBuf[8];
}can_parameter;



#define FILTER_BUF_LEN		5
/*接收到的电机参数结构体*/
typedef struct{
	  int16_t	 	speed_rpm;
    int16_t  	real_current;
    int16_t  	given_current;
    uint8_t  	hall;
	  uint16_t 	angle;				//abs angle range:[0,8191]
	  uint16_t 	last_angle;	//abs angle range:[0,8191]
	  uint16_t	offset_angle;
   	int32_t		round_cnt;
	  int32_t		total_angle;
	  u8			buf_idx;
	  u16			angle_buf[FILTER_BUF_LEN];
	  u16			fited_angle;
	  u32			msg_cnt;
	  float   real_angle;			// 0~无穷大
}moto_measure_t;


union Point_Data
{
	u8 data[8];
	s16 s_data[4];
	float f_data[2];
};



extern can_parameter canDat;
extern moto_measure_t motoS_chassis; 


u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
u8 CAN1_Send_Msg(s16 iq1, s16 iq2, s16 iq3, s16 iq4);	
u8 CAN2_Send_Msg(s16 iq1, s16 iq2, s16 iq3, s16 iq4);	

void get_moto_offset(moto_measure_t *ptr, u8 *buf);
void get_moto_measure_3508(moto_measure_t *ptr, u8 *buf);
void get_moto_measure_2006(moto_measure_t *ptr, u8 *buf);



#endif
