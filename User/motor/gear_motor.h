#ifndef __GEAR_MOTOR_H
#define __GEAR_MOTOR_H

#include "stm32f10x.h"


#define GEARMOTOR_GPIO_PORT    		GPIOE			              /* GPIO�˿� */
#define GEARMOTOR_GPIO_CLK 	    	RCC_APB2Periph_GPIOE		/* GPIO�˿�ʱ�� */

#define GEARMOTOR_GPIO_PIN1				GPIO_Pin_0		       
#define GEARMOTOR_GPIO_PIN2				GPIO_Pin_1			       
#define GEARMOTOR_GPIO_PIN3				GPIO_Pin_2			     

#define	digitalHi(p,i)		 {p->BSRR=i;}	 						//���Ϊ�ߵ�ƽ		
#define digitalLo(p,i)		 {p->BRR=i;}	 						//����͵�ƽ


#define FWD		   		 {digitalHi(GEARMOTOR_GPIO_PORT,GEARMOTOR_GPIO_PIN1);digitalLo(GEARMOTOR_GPIO_PORT,GEARMOTOR_GPIO_PIN2);digitalHi(GEARMOTOR_GPIO_PORT,GEARMOTOR_GPIO_PIN3);}
#define REV			     {digitalLo(GEARMOTOR_GPIO_PORT,GEARMOTOR_GPIO_PIN1);digitalHi(GEARMOTOR_GPIO_PORT,GEARMOTOR_GPIO_PIN2);digitalHi(GEARMOTOR_GPIO_PORT,GEARMOTOR_GPIO_PIN3);}
#define STOP			   {digitalLo(GEARMOTOR_GPIO_PORT,GEARMOTOR_GPIO_PIN1);digitalLo(GEARMOTOR_GPIO_PORT,GEARMOTOR_GPIO_PIN2);}

void GearMotor_Config(void);



#endif   /*__GEAR_MOTOR_H*/

