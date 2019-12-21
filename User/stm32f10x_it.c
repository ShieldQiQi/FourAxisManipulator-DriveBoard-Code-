/*******************************************************************
*	Date:2019-03-23
*	Author:SHIELD_QIQI
*	Project:six-axis manipulator
*****************************************/

#include "stm32f10x_it.h"
#include "usart.h"
#include "blue_tooth.h"
#include "TiMbase.h"
#include "led.h"
#include "adc.h"
#include "beep.h"
#include "AdvanceTim_PWM.h"
#include "msg_queue.h"

uint8_t     tmp =0;
uint16_t    time0 = 0;
uint8_t     time1 = 0;
uint8_t			isTrueDataFlag = 0;
uint8_t			count = 0;

uint8_t			receiveData = 0;
uint8_t			msgBuffer[6];
extern 			LinkQueue thetaArray_Queue;


extern __IO uint16_t 		ADC_ConvertedValue[NOFCHANEL]; 	 
float 									ADC_ConvertedValueLocal[NOFCHANEL]; 



/*------------------------------------------------------------------------------------------------------------------------------------------------
定时中断：：100HZ / 10ms
-------------------------------------------------------------------------------------------------------------*/

void  BASIC_TIM_IRQHandler (void)
{
	if ( TIM_GetITStatus( BASIC_TIM, TIM_IT_Update) != RESET ) 
	{	
		
		//--------------------------------------
		if(time1 % 15 == 0)
		{
			LED1_TOGGLE;
			LED2_OFF;
			
			ADC_ConvertedValueLocal[0] =(float) ADC_ConvertedValue[0]/4096*3.3;
			ADC_ConvertedValueLocal[1] =(float) ADC_ConvertedValue[1]/4096*3.3;
			ADC_ConvertedValueLocal[2] =(float) ADC_ConvertedValue[2]/4096*3.3;
			ADC_ConvertedValueLocal[3] =(float) ADC_ConvertedValue[3]/4096*3.3;
			ADC_ConvertedValueLocal[4] =(float) ADC_ConvertedValue[4]/4096*3.3;
			ADC_ConvertedValueLocal[5] =(float) ADC_ConvertedValue[5]/4096*3.3;
		
		}
			
		//--------------------------------------
		if(time1 % 30 == 0)
		{	
			if(!is_QueueEmpty(thetaArray_Queue))
				if(count != 6){
					//Set_Angle(count+1,(uint16_t)(thetaArray_Queue.front->next->theta[count]));
					USART_SendData(Bluetooth_USART1, (uint16_t)(thetaArray_Queue.front->next->theta[count]));
					count++;
				}else{
					count = 0;
					USART_SendData(Bluetooth_USART1, 255);
					pop(&thetaArray_Queue);
				}
		}
		
		//--------------------------------------
		if(time0==150)
		{
			time0 = 0;
		}
		
		
		//--------------------------------------
		if(time1 == 240)
			time1 = 0;
		
		time0++;
		time1++;
		
		TIM_ClearITPendingBit(BASIC_TIM , TIM_FLAG_Update);  		 
	}
}


/*------------------------------------------------------------------------------------------------------------------------------------------------
定时中断：：100HZ / 10ms
接收树莓派串口消息
-------------------------------------------------------------------------------------------------------------*/

void RasberryPi_USART_IRQHandler(void) 
{
 
	if(USART_GetITStatus(RasberryPi_USART3,USART_IT_RXNE)!=RESET)
	{	
		receiveData = USART_ReceiveData(RasberryPi_USART3);
		LED2_ON;
		//接收报文内容
		if(isTrueDataFlag)
		{
			msgBuffer[isTrueDataFlag-1] = receiveData;
			
			isTrueDataFlag++;
			if(isTrueDataFlag == 7){
				Push(&thetaArray_Queue,msgBuffer);
				isTrueDataFlag = 0;
			}
		}
		
		//接收报文头
		if(receiveData == 255){
			isTrueDataFlag = 1;
		}
		
	}
}

/*------------------------------------------------------------------------------------------------------------------------------------------------
定时中断：：100HZ / 10ms
接收蓝牙串口消息
-------------------------------------------------------------------------------------------------------------*/

void Bluetooth_USART1_IRQHandler(void) 
{
  uint8_t ucTemp=1;
	if(USART_GetITStatus(Bluetooth_USART1,USART_IT_RXNE)!=RESET)
	{
		ucTemp=USART_ReceiveData(Bluetooth_USART1);
		if(ucTemp==48){
			beep_ON;
		}
		else if(ucTemp==49){
			beep_OFF;
		}
	}
}

/**************************************************************END OF FILE**************************************************************/
