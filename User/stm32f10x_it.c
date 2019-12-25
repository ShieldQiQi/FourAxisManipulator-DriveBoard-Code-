/*******************************************************************
*	Date:2019-03-23
*	Author:SHIELD_QIQI
*	Project:six-axis manipulator
*****************************************/

#include <math.h>
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

uint8_t     i =0;
uint8_t     m =0;
uint8_t     tmp =0;
uint8_t     restCount =0;
uint8_t     triggerBeep =0;
uint16_t    time0 = 0;
uint8_t     time1 = 0;
uint16_t    time2 = 0;
uint8_t     time3 = 0;
uint8_t     is_nextText = 0;
uint8_t     is_meetEnd = 0;
uint8_t			isTrueDataFlag = 0;
uint8_t			onWriting = 0;
uint8_t			count = 0;
uint8_t			sendFlag = 0;		
uint8_t			receiveData = 0;
uint16_t		msgBuffer[6];
extern 			LinkQueue thetaArray_Queue;

extern float						Position_KP,Position_KI,Position_KD;  
extern float		 				Velocity1,Velocity2,Velocity3,Velocity4,Velocity5,Velocity6;
extern float 						Position1,Position2,Position3,Position4,Position5,Position6;
float										currentAngles[6] = {0,0,0,0,0,0};
float										deltaMax = 0;
float										time_finish = 0;
float										time_used = 0;
float										coefficients[6][4];

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
			if(!is_QueueEmpty(thetaArray_Queue) && (time3 == 27 || time3 == 0)){
				
				time3 = 0;
				if(restCount < 5)
				{
					//碰撞检测、安全措施
					if(thetaArray_Queue.front->next->theta[0]-180 >= -90 && thetaArray_Queue.front->next->theta[0]-180 <= 90 &&
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
						//找出当前值与目标值最大的角度
						deltaMax = deltaMax < abs(thetaArray_Queue.front->next->theta[0]-180+129 - currentAngles[0]) ? abs(thetaArray_Queue.front->next->theta[0]-180+129 - currentAngles[0]):deltaMax;
						deltaMax = deltaMax < abs(thetaArray_Queue.front->next->theta[1]-180+225 - currentAngles[1]) ? abs(thetaArray_Queue.front->next->theta[1]-180+225 - currentAngles[1]):deltaMax;
						deltaMax = deltaMax < abs(-thetaArray_Queue.front->next->theta[2]+180+130 - currentAngles[2]) ? abs(-thetaArray_Queue.front->next->theta[2]+180+130 - currentAngles[2]):deltaMax;
						deltaMax = deltaMax < abs(thetaArray_Queue.front->next->theta[3]-180+90 - currentAngles[3]) ? abs(thetaArray_Queue.front->next->theta[3]-180+90 - currentAngles[3]):deltaMax;
						//根据最大的差值和舵机在相应电压下的响应速度分配时间,TBS 2701理论0.16sec/60degree
						time_finish = deltaMax/60*1.6;
						
						if(time_finish - time_used <= 0.01 || onWriting){
							Set_Angle(1,thetaArray_Queue.front->next->theta[0]-180+129);
							Set_Angle(2,thetaArray_Queue.front->next->theta[1]-180+225);
							Set_Angle(3,-thetaArray_Queue.front->next->theta[2]+180+40+90);
							Set_Angle(4,thetaArray_Queue.front->next->theta[3]-180+90);
							Set_Angle(5,135);
							Set_Angle(6,135);
							
							currentAngles[0] = thetaArray_Queue.front->next->theta[0]-180+129;
							currentAngles[1] = thetaArray_Queue.front->next->theta[1]-180+225;
							currentAngles[2] = -thetaArray_Queue.front->next->theta[2]+180+40+90;
							currentAngles[3] = thetaArray_Queue.front->next->theta[3]-180+90;
							currentAngles[4] = 135;
							currentAngles[5] = 135;
							
							time_used = 0;
							pop(&thetaArray_Queue);
							onWriting = 1;
						}else
						{
							if(time_used == 0){
								//使用三次样条对关节空间轨迹进行插值，至少保证初末位置速度为0
								for(i = 0;i<4;i++){
									coefficients[i][0] = currentAngles[i];
									coefficients[i][1] = 0;
								}
								coefficients[0][2] = 3/(time_finish*time_finish)*(thetaArray_Queue.front->next->theta[0]-180+129 - currentAngles[0]);
								coefficients[0][3] = -2/(time_finish*time_finish*time_finish)*(thetaArray_Queue.front->next->theta[0]-180+129 - currentAngles[0]);
								coefficients[1][2] = 3/(time_finish*time_finish)*(thetaArray_Queue.front->next->theta[1]-180+225 - currentAngles[1]);
								coefficients[1][3] = -2/(time_finish*time_finish*time_finish)*(thetaArray_Queue.front->next->theta[1]-180+225 - currentAngles[1]);
								coefficients[2][2] = 3/(time_finish*time_finish)*(-thetaArray_Queue.front->next->theta[2]+180+130 - currentAngles[2]);
								coefficients[2][3] = -2/(time_finish*time_finish*time_finish)*(-thetaArray_Queue.front->next->theta[2]+180+130 - currentAngles[2]);			
								coefficients[3][2] = 3/(time_finish*time_finish)*(thetaArray_Queue.front->next->theta[3]-180+90 - currentAngles[3]);
								coefficients[3][3] = -2/(time_finish*time_finish*time_finish)*(thetaArray_Queue.front->next->theta[3]-180+90 - currentAngles[3]);
							}
							//每隔30ms进行一次插值
							for(i = 0;i<4;i++){
								time_used += 0.01;
								Set_Angle(i+1,coefficients[i][0]+coefficients[i][1]*time_used+coefficients[i][2]*time_used*time_used+coefficients[i][3]*time_used*time_used*time_used);
							}
						}
#endif
						triggerBeep = 0;
						beep_OFF;
					}else//触发蜂鸣器报警
					{
						triggerBeep = 1;
					}
				}else{
					m = 0;
					if(restCount <= 7){
						restCount++;
						Set_Angle(2,115);
						Set_Angle(3,80);
						Set_Angle(4,90);
						
						currentAngles[1] = 115;
						currentAngles[2] = 80;
						currentAngles[3] = 90;
					}else if(restCount <= 11)
					{
						restCount++;
						Set_Angle(1,129);
						currentAngles[0] = 129;
					}else{
						restCount = 0;
					}
					
				}
				//写完一个字、移动宣纸位置
				if(is_QueueEmpty(thetaArray_Queue)){
					is_nextText = 1;
					onWriting = 0;
				}
			}else{
				if(restCount <=2)
				{
					Set_Angle(2,115);
					Set_Angle(3,80);
					Set_Angle(4,90);
					
					currentAngles[1] = 115;
					currentAngles[2] = 80;
					currentAngles[3] = 90;
					restCount++;
				}else if(restCount <=4)
				{
					Set_Angle(1,129+90);
					Set_Angle(2,135);
					Set_Angle(3,60);
					
					currentAngles[0] = 129+90;
					currentAngles[1] = 135;
					currentAngles[2] = 60;				
					restCount++;
				}else if(restCount ==5){
					if(m < 20){
						m++;
					}else{
						currentAngles[2] = 40;	
					}
					Set_Angle(3,60-m);
				}
				if(time3 < 27)
					time3++;
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
			if(time2 != 500){
				FWD;
				time2++;
			}else if(time2 == 500){
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
			}else if(time2 == 2600){
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
