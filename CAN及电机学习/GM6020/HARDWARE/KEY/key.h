#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 


/*下面的方式是通过直接操作库函数方式读取IO*/
#define KEY1 		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)	//PB0
#define KEY2 		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) //PB1

#define KEY1_PRES	1
#define KEY2_PRES	2

void KEY_Init(void);	//IO初始化
uint8_t KEY_Scan(uint8_t);  		//按键扫描函数	

#endif
