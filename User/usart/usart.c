/*******************************************************************
*	Date:2019-03-23
*	Author:SHIELD_QIQI
*	Project:E:six-axis manipulator
*****************************************/ 
	
#include "usart.h"

//串口初始化配置
void USART3_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(RasberryPi_USART3, &USART_InitStructure);
	
	USART_ITConfig(RasberryPi_USART3, USART_IT_RXNE, ENABLE);	
	
	USART_Cmd(RasberryPi_USART3, ENABLE);		

  // 清除发送完成标志
	//USART_ClearFlag(USART1, USART_FLAG_TC);     
}

//发送一个字符
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{

	USART_SendData(pUSARTx,ch);
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

//发送字符串
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

//发送16位数
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	temp_h = (ch&0XFF00)>>8;
	temp_l = ch&0XFF;
	
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	USART_SendData(RasberryPi_USART3, (uint8_t) ch);
	
	while (USART_GetFlagStatus(RasberryPi_USART3, USART_FLAG_TXE) == RESET);		

	return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		while (USART_GetFlagStatus(RasberryPi_USART3, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(RasberryPi_USART3);
}

/************************END OF FILE********************/

