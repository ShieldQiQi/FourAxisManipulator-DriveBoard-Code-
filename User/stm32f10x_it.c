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
#include "control.h"	
#include "gear_motor.h"

uint8_t     tmp =0;
uint8_t     triggerBeep =0;
uint16_t    time0 = 0;
uint8_t     time1 = 0;
uint16_t    time2 = 0;
uint8_t     is_nextText = 0;
uint8_t     is_meetEnd = 0;
uint8_t			isTrueDataFlag = 0;
uint8_t			count = 0;
uint8_t			sendFlag = 0;		
uint8_t			receiveData = 0;
uint16_t		msgBuffer[6];
extern 			LinkQueue thetaArray_Queue;

extern float						Position_KP,Position_KI,Position_KD;  
extern float		 				Velocity1,Velocity2,Velocity3,Velocity4,Velocity5,Velocity6;
extern float 						Position1,Position2,Position3,Position4,Position5,Position6;

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
		if(time1 % 15 == 0)
		{
			if(!is_QueueEmpty(thetaArray_Queue)){
				//碰撞检测、安全措施
				if(thetaArray_Queue.front->next->theta[0]-180 > -90 && thetaArray_Queue.front->next->theta[0]-180 < 90 &&
					thetaArray_Queue.front->next->theta[1]-180 < 0 && thetaArray_Queue.front->next->theta[1]-180 > -150 &&
					thetaArray_Queue.front->next->theta[2]-180 > 0 && thetaArray_Queue.front->next->theta[2]-180 < 150){
						
#if	USE_PID == 1	
					Velocity1=Position_PID1(Position1,thetaArray_Queue.front->next->theta[0]);
					Velocity2=Position_PID2(Position2,thetaArray_Queue.front->next->theta[1]);
					Velocity3=Position_PID3(Position3,thetaArray_Queue.front->next->theta[2]);
					Velocity4=Position_PID4(Position4,thetaArray_Queue.front->next->theta[3]);
					Velocity5=Position_PID5(Position5,thetaArray_Queue.front->next->theta[4]);
					Velocity6=Position_PID6(Position6,thetaArray_Queue.front->next->theta[5]);
						
					Set_Pwm(Velocity1,Velocity2,Velocity3,Velocity4,Velocity5,Velocity6);
#else					
					Set_Angle(1,thetaArray_Queue.front->next->theta[0]-180+129);
					Set_Angle(2,thetaArray_Queue.front->next->theta[1]-180+225);
					Set_Angle(3,-thetaArray_Queue.front->next->theta[2]+180+40+90);
					Set_Angle(4,thetaArray_Queue.front->next->theta[3]-180+90);
					Set_Angle(5,135);
					Set_Angle(6,135);
#endif
					
					triggerBeep = 0;
					beep_OFF;
				}else//触发蜂鸣器报警
				{
					triggerBeep = 1;
				}
				
				pop(&thetaArray_Queue);
				
				//写完一个字、移动宣纸位置
				if(is_QueueEmpty(thetaArray_Queue))
					is_nextText = 1;
			}else{
				Set_Angle(1,129);
				Set_Angle(2,115);
				Set_Angle(3,80);
				Set_Angle(4,90);
				Set_Angle(5,80);
				Set_Angle(6,100);
				triggerBeep = 0;
				beep_OFF;
			}
		}
		
		//--------------------------------------
		//碰撞检测，危险报警
		if(triggerBeep){
			if(time0 % 10 == 0){
				beep_ON;
			}else if(time0 % 1 == 0){
				beep_OFF;
			}
		}
		
		//--------------------------------------
		//写完一个字、移动宣纸
		if(is_nextText && is_meetEnd != 5 )
		{
			if(time2 != 400){
				FWD;
				time2++;
			}else if(time2 == 400){
				STOP;
				is_nextText = 0;
				is_meetEnd ++;
				time2 = 0;
				//告知主机已经写完一个字
				USART_SendData(RasberryPi_USART3,0);
			}
		}else if(is_meetEnd != 5){
			time2 = 0;
		}
		//写到尽头、回起始点换纸
		if(is_meetEnd == 5)
		{
			if(time2 == 0){
				STOP;
			}else if(time2 == 100){
				REV;
			}else if(time2 == 2100){
				STOP;
				is_meetEnd = 0;
			}
			time2++;
		}
		
		//--------------------------------------
		if(time0==150)
		{
			time0 = 0;
		}
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
		if(isTrueDataFlag >= 2)
		{
			if(isTrueDataFlag % 2 == 0)	//取低8位
				msgBuffer[isTrueDataFlag/2-1] = receiveData;
			else												//取高8位
				msgBuffer[isTrueDataFlag/2-1] = (((uint16_t)receiveData) << 8) | (msgBuffer[isTrueDataFlag/2-1] & 0x00FF);
			
			isTrueDataFlag++;
			if(isTrueDataFlag == 14){
				Push(&thetaArray_Queue,msgBuffer);
				isTrueDataFlag = 0;
			}
		}
		
		//接收报文头
		if(receiveData == 255 && isTrueDataFlag < 2){
			isTrueDataFlag++;
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
