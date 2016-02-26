#ifndef _HJ5000_SCANNER_H_
#define _HJ5000_SCANNER_H_
#include "stm32f10x_lib.h"

#define SCANNER_TRIG_HW		//Ó²¼þIO´¥·¢

#define USART_RX_DMA_MODE		1
#define USART_RX_ISR_MODE		2

#define USART_RX_MODE		USART_RX_DMA_MODE
//#define USART_RX_MODE		USART_RX_ISR_MODE

#define G_SEND_BUF_LENGTH     32
#define G_RECEIV_BUF_LENGTH   128

extern unsigned char		g_send_buff[G_SEND_BUF_LENGTH];
extern unsigned char		g_receive_buff[G_RECEIV_BUF_LENGTH];


#define SCANNER_TRIG_ON()	GPIO_ResetBits(GPIOB,GPIO_Pin_12);	
#define SCANNER_TRIG_OFF()	GPIO_SetBits(GPIOB,GPIO_Pin_12);	

void reset_resVar(void);
void reset_resVar_ext(void);
int scanner_RxISRHandler(unsigned char *c,int len);
void scanner_mod_init(void);
void scanner_mod_reset(void);
int scanner_get_barcode(unsigned char *barcode,unsigned int max_num,unsigned char *barcode_type,unsigned int *barcode_len);
#endif