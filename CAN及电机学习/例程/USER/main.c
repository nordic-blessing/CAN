#include "systemInit.h"

int main(void)
{ 
	system_Init();
	while(1)
	{	
		//LED1 = !LED1;
		//LED0 = !LED1;
		printf("s:%d,%d\r\n",motoS_chassis.speed_rpm,5000);  //Œª÷√ª∑¥Ú”°
		delay_ms(100);		
	}
}

