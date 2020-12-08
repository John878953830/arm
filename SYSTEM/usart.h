#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
	  	
void uart_init(uint32_t bound);
char USART1_Read(void);
void USART1_Send(char ch);
void USART1_SendStr(uint8_t *pData, uint16_t Len);
#endif


