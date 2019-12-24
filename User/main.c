/*******************************************************************
*	Date:2019-03-23
*	Author:SHIELD_QIQI
*	Project:E:six-axis manipulator
*****************************************/ 

#include "stm32f10x.h"
#include "usart.h"
#include "blue_tooth.h"
#include "nvic.h"
#include "led.h"
#include "TiMbase.h"
#include "AdvanceTim_PWM.h"
#include "adc.h"
#include "beep.h"
#include "msg_queue.h"
#include "gear_motor.h"

int main(void)
{	
	GearMotor_Config();
  USART3_Config();
	USART1_Config();
	LED_GPIO_Config();
	BASIC_TIM_Init();
	ADVANCE_TIM_Init();
	ADCx_Init();
	BEEP_Config();
  NVIC_Configuration();
	
	Init_usartMsg_Queue();
	
	Set_Angle(1,129);
	Set_Angle(2,115);
	Set_Angle(3,40);
	Set_Angle(4,90);
	Set_Angle(5,80);
	Set_Angle(6,100);
	while(1)
	{
		
	}
	
}


/*******************END OF FILE**********************/
