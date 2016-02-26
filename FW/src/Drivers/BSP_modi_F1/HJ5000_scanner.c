/**
* @file  HJ5000_scanner.c
* @brief HJ5000红光CCD条码扫描引擎的驱动模块
* @version 1.0
* @author joe.zhou
* @date 2015年08月22日
* @note
* 
*/
#include <string.h>
#include "stm32f10x_lib.h"
#include "ucos_ii.h"
#include "HJ5000_scanner.h"
#include "TimeBase.h"
#include "keypad.h"
#include "PCUsart.h"

#define HJ5000_FRAME_MAX_LEN		257

#define USART_RX_DMA_MODE		1
#define USART_RX_ISR_MODE		2

//#define USART_RX_MODE		USART_RX_DMA_MODE
#define USART_RX_MODE		USART_RX_ISR_MODE


#define HJ5000_START_DECODE		1
#define HJ5000_STOP_DECODE		2
#define HJ5000_READ_CFG			3
#define HJ5000_SET_ALL_DEFAULT	4


#define ENABLE		1
#define DISABLE		0

#define HJ5000_START_DECODE		1
#define HJ5000_STOP_DECODE		2
#define HJ5000_READ_CFG			3
#define HJ5000_SET_ALL_DEFAULT	4


#define ENABLE		1
#define DISABLE		0

	
#define InterfaceType	0x00	//00H KB  01H RS232   02H USB_KB  03H   USB_RS232 
#define KBType			0x01	//00H	键盘类型
#define KBLanguage		0x02	//00H 	键盘语言
#define DspCodeMark		0x03	//00H	显示所有条形码代码
#define KBWedge			0x04	//00H	键盘功能设置		
#define KBEmulation		0x05	//00H	仿真键盘响应(for notebook)		
#define KBCLKSpeed		0x06	//10H	Clock(实际频率)=20+KBCLKSpeed. Default=0x10.		
#define InterCharDel	0x07	//00H	0 ~ 99mS (输入值乘  1mS)		
#define InterMessDel	0x08	//00H	0 ~  5 S (输入值乘100mS)		
#define KBTerminator	0x09	//01H	00H=NONE, 01H=0DH, 02H=09H.		
#define RSFlowCtrl		0x0A	//00H	00H=NONE, 01H=CTS/RTS,02H=Xon/Xoff.		
#define RSHshakingDel	0x0B	//00H	Handshaking Delay		0x0C				
#define RSACKDel		0x0D	//00H			
#define RSParity		0x0E	//00H	(00H=None, 01H=Even, 02H=Odd) 01H Without odd		
#define RSStop			0x0F	//00H	(00H=1 Stop Bit, 01H=2 Stop Bit)		
#define RSData			0x10	//01H	(00H=7 Data Bits, 01H=8 Data Bits)		
#define RSBaudRate		0x11	//04H	(00H=9600, 01H=9600, 02H=2400, 03H=4800, 04H=9600 , 05H=19200, 06H=57600, 07H=38400, 08H=115200)		
#define RSTerminator	0x12	//01H	(00H=NONE, 01H=0DH&0AH, 02H=0DH, 03H=0AH, 04H=09H, 05H=03H, 06H=04H)		
#define FlashSW			0x13	//00H	00H=长亮模式时不闪烁, 01H=长亮模式时闪烁		
		
#define GoodReadBeep	0x17	//01H	读取声, 00H=Disable, 01H=Enable		
#define PowerUpTone		0x18	//01H	开机声, 00H=Disable, 01H=Enable		
#define ScanMode		0x19	//01H	00H=Trigger On/Off, 01H=Trigger On/Good read off, 02H=Continuous/Trigger off, 04H=continuous/LED always ON, 05H=Continuous/timeout off, 06=Flash ON, 07=Continuous/No Trigger, 08=Software trigger and trigger pin=Low,0A=Software trigger		
#define SoundFre		0x1A	//09H	0~15音频对应(1.5KHz~3KHz)		
#define SoundLeng		0x1B	//04H	0~99 mS		
#define PrefixChar_L	0x1C	//00H	 Prefix目前输入字数		
#define PrefixChar		0x1D	//00H	1DH~26H位置数据		

#define PostfixChar_L	0x27	//00H	Postfix目前输入字数		
#define PostfixChar		0x28	//00H	28H~31H位置数据		
					
#define UAEnable		0x32	//01H	// UPC-A     ON/OFF ( 00H=Disable, 01H=Enable)		
#define UAIdent			0x33	//46H	// UPC-A     ID code		
#define UAInsertCode	0x34	//00H	// UPA-A     insert G1 or G2 code ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define UAtoE13			0x35	//00H	// UPC-A     convert to EAN13  ( 00H=Disable, 01H=Enable)		
#define UATranCheck		0x36	//01H	// UPC-A     transmit CC(check character)  ( 00H=Disable, 01H=Enable)		
#define UACut0			0x37	//00H	// UPC-A     cut first Zero  ( 00H=Disable, 01H=Enable)		
#define UACutFNum		0x38	//00H	// UPC-A     cut front number		
#define UACutBNum		0x39	//00H	// UPC-A     cut back number		
#define UAAddendum		0x3A	//00H	// UPC-A     Add on 2/5		
#define UAAddForce		0x3B	//00H	// UPC-A     enforce Add on 2/5		
					
#define UEEnable		0x3C	//01H	// UPC-E     ON/OFF ( 00H=Disable, 01H=Enable)		
#define UEIdent			0x3D	//47H	// UPC-E     ID code		
#define UEInsertCode	0x3E	//00H	// UPA-E     insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define UEtoUA			0x3F	//00H	// UPC-E     convert to UPC-A ( 00H=Disable, 01H=Enable)		
#define UETranCheck		0x40	//01H	// UPC-E     transmit CC ( 00H=Disable, 01H=Enable)		
#define UECut0			0x41	//00H	// UPC-E     cut first Zero ( 00H=Disable, 01H=Enable)		
#define UECutFNum		0x42	//00H	// UPC-E     cut front number		
#define UECutBNum		0x43	//00H	// UPC-E     cut back number		
#define UEAddendum		0x44	//00H	// UPC-E     Add on 2/5		
#define UEAddForce		0x45	//00H	// UPC-E     enforce Add on 2/5		
					
#define E13Enable		0x46	//01H	// EAN13     ON/OFF ( 00H=Disable, 01H=Enable)		
#define E13Ident		0x47	//48H	// EAN13     ID code		
#define E13InsertCode	0x48	//00H	// EAN13     insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define E13TranCheck	0x49	//01H	// EAN13     transmit CC ( 00H=Disable, 01H=Enable)		
#define E13CutFNum		0x4A	//00H	// EAN13     cut front number		
#define E13CutBNum		0x4B	//00H	// EAN13     cut back number		
#define E13Addendum		0x4C	//00H	// EAN13     Add on 2/5		
#define E13AddForce		0x4D	//00H	// EAN13     enforce Add on 2/5		
#define E13ISEnable		0x4E	//00H	// ISBNISSN  exchange ( 00H=Disable, 01H=Enable)		
#define E13ISBNAdd		0x4F	//00H	// ISBN      insert 间隔符号 ( 00H=Disable, 01H=Enable)		
					
#define E8Enable		0x50	//01H	// EAN-8     ON/OFF ( 00H=Disable, 01H=Enable)		
#define E8Ident			0x51	//49H	// EAN-8     ID code		
#define E8InsertCode	0x52	//00H	// EAN-8     insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define E8TranCheck		0x53	//01H	// EAN-8     transmit CC ( 00H=Disable, 01H=Enable)		
#define E8CutFNum		0x54	//00H	// EAN-8     cut front number		
#define E8CutBNum		0x55	//00H	// EAN-8     cut back number		
#define E8Addendum		0x56	//00H	// EAN-8     Add on 2/5		
#define E8AddForce		0x57	//00H	// EAN-8     enforce Add on 2/5		
					
#define Addendum		0x58	//00H	// UPC_EAN   Add on 2/5  (same time open UPC/EAN all add on 2/5)		
#define AddendForce		0x59	//00H	// UPC_EAN   enforce Add on 2/5 (同时强制UPC/EAN所有附加码)		
					
#define C39Enable		0x5A	//01H	// Code39    ON/OFF ( 00H=Disable, 01H=Enable)		
#define C39Ident		0x5B	//4AH	// Code39    ID code		
#define C39InsertCode	0x5C	//00H	// Code39    insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define C39Check		0x5D	//00H	// Code39    Check CS ( 00H=Disable, 01H=Enable)		
#define C39TranCheck	0x5E	//00H	// Code39    transmit CC ( 00H=Disable, 01H=Enable)		
#define C39Concat		0x5F	//00H	// Code39    当C39第一字符是"space"时,close message terminator transmit ( 00H=Disable, 01H=Enable)		
#define C39Format		0x60	//00H	// Code39    格式(standard or Full code) ( 00H=Standard C39, 01H=Full C39)		
#define C39TranOEChar	0x61	//00H	// Code39    传送起始&结束符号 ( 00H=Disable, 01H=Enable)		
#define C39CutFNum		0x62	//00H	// Code39    cut front number		
#define C39CutBNum		0x63	//00H	// Code39    cut back number		
#define C39SLeng		0x64	//00H	// Code39    min length		
#define C39DLeng		0x65	//50H	// Code39    max length		
					
#define CdaEnable		0x66	//01H	// Codabar   ON/OFF ( 00H=Disable, 01H=Enable)		
#define CdaIdent		0x67	//4BH	// Codabar   ID code		
#define CdaInsertCode	0x68	//00H	// Codabar   insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define CdaCheck		0x69	//00H	// Codabar   Check CS ( 00H=Disable, 01H=Enable)		
#define CdaTranCheck	0x6A	//01H	// Codabar   transmit CC ( 00H=Disable, 01H=Enable)		
#define CdaConcat		0x6B	//00H	//		
#define CdaOECharType	0x6C	//00H	// Codabar   传送起始&结束类型(00H=ABCD/ABCD,01H=abcd/abcd,02H=abcd/tn*e)		
#define CdaTranOEChar	0x6D	//01H	// Codabar   传送起始&结束符号 ( 00H=Disable, 01H=Enable)		
#define CdaCutFNum		0x6E	//00H	// Codabar   cut front number		
#define CdaCutBNum		0x6F	//00H	// Codabar   cut back number		
#define CdaSLeng		0x70	//06H	// Codabar   min length		
#define CdaDLeng		0x71	//50H	// Codabar   max length		
					
#define C93Enable		0x72	//01H	// Code93    ON/OFF ( 00H=Disable, 01H=Enable)		
#define C93Ident		0x73	//4CH	// Code93    ID code		
#define C93InsertCode	0x74	//00H	// Code93    insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define C93Check		0x75	//01H	// Code93    Check CS ( 00H=Disable, 01H=Enable)		
#define C93TranCheck	0x76	//00H	// Code93    transmit CC ( 00H=Disable, 01H=Enable)		
#define C93CutFNum		0x77	//00H	// Code93    cut front number		
#define C93CutBNum		0x78	//00H	// Code93    cut back number		
#define C93SLeng		0x79	//03H	// Code93    min length		
#define C93DLeng		0x7A	//50H	// Code93    max length		
					
#define C128Enable		0x7B	//01H	// Code128   ON/OFF ( 00H=Disable, 01H=Enable)		
#define C128Ident		0x7C	//4DH	// Code128   ID code		
#define C128InsertCode	0x7D	//00H	// Code128   insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define C128Check		0x7E	//01H	// Code128   Check CS ( 00H=Disable, 01H=Enable)		
#define C128TranCheck	0x7F	//00H	// Code128   transmit CC ( 00H=Disable, 01H=Enable)		
#define C128Fuc1		0x80	//00H	// Code128    当C128第一字符是"FUC1"时,结合下一组条形码 message ( 00H=Disable, 01H=Enable)		
#define C128Fuc2		0x81	//00H	// Code128    当C128中有"FUC2"时, FUC2转换为"1DH" ( 00H=Disable, 01H=Enable)		
#define EAN128En		0x82	//00H	// Code128 转换为EAN128 ( 00H=Disable, 01H=Enable)		
#define C128CutFNum		0x83	//00H	// Code128   cut front number		
#define C128CutBNum		0x84	//00H	// Code128   cut back number		
#define C128SLeng		0x85	//03H	// Code128   min length		
#define C128DLeng		0x86	//50H	// Code128   max length		
					
#define It25Enable		0x87	//01H	// It25      ON/OFF ( 00H=Disable, 01H=Enable)		
#define It25Ident		0x88	//4EH	// It25      ID code		
#define It25InsertCode	0x89	//00H	// It25      insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define It25Check		0x8A	//00H	// It25      Check CS ( 00H=Disable, 01H=Enable)		
#define It25TranCheck	0x8B	//00H	// It25      transmit CC ( 00H=Disable, 01H=Enable)		
#define It25CutFNum		0x8C	//00H	// It25      cut front number		
#define It25CutBNum		0x8D	//00H	// It25      cut back number		
#define It25SLeng		0x8E	//04H	// It25      min length		
#define It25DLeng		0x8F	//50H	// It25      max length		
					
#define Id25Enable		0x90	//00H	// Id25      ON/OFF ( 00H=Disable, 01H=Enable)		
#define Id25Ident		0x91	//4FH	// Id25      ID code		
#define Id25InsertCode	0x92	//00H	// Id25      insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define Id25Check		0x93	//00H	// Id25      Check CS ( 00H=Disable, 01H=Enable)		
#define Id25TranCheck	0x94	//00H	// Id25      transmit CC ( 00H=Disable, 01H=Enable)		
#define Id25CutFNum		0x95	//00H	// ID25      cut front number		
#define Id25CutBNum		0x96	//00H	// ID25      cut back number		
#define Id25SLeng		0x97	//06H	// Id25      min length		
#define Id25DLeng		0x98	//50H	// Id25      max length		
					
#define S25Enable		0x99	//00H	// S25       ON/OFF ( 00H=Disable, 01H=Enable)		
#define S25Ident		0x9A	//57H	// S25       ID code		
#define S25InsertCode	0x9B	//00H	// S25       insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define S25CutFNum		0x9C	//00H	// S25       cut front number		
#define S25CutBNum		0x9D	//00H	// S25       cut back number		
#define S25SLeng		0x9E	//06H	// S25       min length		
#define S25DLeng		0x9F	//50H	// S25       max length		
					
#define M25Enable		0xA0	//00H	// M25       ON/OFF ( 00H=Disable, 01H=Enable)		
#define M25Ident		0xA1	//50H	// M25       ID code		
#define M25InsertCode	0xA2	//00H	// M25       insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define M25Check		0xA3	//00H	// M25       Check CS ( 00H=Disable, 01H=Enable)		
#define M25TranCheck	0xA4	//01H	// M25       transmit CC ( 00H=Disable, 01H=Enable)		
#define M25CutFNum		0xA5	//00H	// M25       cut front number		
#define M25CutBNum		0xA6	//00H	// M25       cut back number		
#define M25SLeng		0xA7	//06H	// M25       min length		
#define M25DLeng		0xA8	//50H	// M25       max length		
					
#define CPEnable		0xA9	//00H	// CPC       ON/OFF ( 00H=Disable, 01H=Enable)		
#define CPIdent			0xAA	//51H	// CPC       ID code		
#define CPInsertCode	0xAB	//00H	// CPC       insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define CPCheck			0xAC	//00H	// CPC       Check CS ( 00H=Disable, 01H=Enable)		
#define CPTranCheck		0xAD	//00H	// CPC       transmit CC ( 00H=Disable, 01H=Enable)		
#define CPCutFNum		0xAE	//00H	// CPC       cut front number		
#define CPCutBNum		0xAF	//00H	// CPC       cut back number		
#define CPSLeng			0xB0	//06H	// CPC       min length		
#define CPDLeng			0xB1	//50H	// CPC       max length		
					
#define C11Enable		0xB2	//00H	// Code11    ON/OFF ( 00H=Disable, 01H=Enable)		
#define C11Ident		0xB3	//54H	// Code11    ID code		
#define C11InsertCode	0xB4	//00H	// Code11    insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define C11Check		0xB5	//00H	// Code11    Check CS ( 00H=Disable, 01H=Enable)		
#define C11TranCheck	0xB6	//00H	// Code11    transmit CC ( 00H=Disable, 01H=Enable)		
#define C11CutFNum		0xB7	//00H	// Code11    cut front number		
#define C11CutBNum		0xB8	//00H	// Code11    cut back number		
#define C11SLeng		0xB9	//06H	// Code11    min length		
#define C11DLeng		0xBA	//50H	// Code11    max length		
					
#define MSEnable		0xBB	//00H	// MSI       ON/OFF ( 00H=Disable, 01H=Enable)		
#define MSIdent			0xBC	//52H	// MSI       ID code		
#define MSInsertCode	0xBD	//00H	// MSI       insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define MSCheck			0xBE	//00H	// MSI       Check CS ( 00H=Disable, 01H=Enable)		
#define MSTranCheck		0xBF	//01H	// MSI       transmit CC ( 00H=Disable, 01H=Enable)		
#define MSCutFNum		0xC0	//00H	// MSI       cut front number		
#define MSCutBNum		0xC1	//00H	// MSI       cut back number		
#define MSSLeng			0xC2	//06H	// MSI       min length		
#define MSDLeng			0xC3	//50H	// MSI       max length		
					
#define C32Enable		0xC4	//00H	// Code32    ON/OFF ( 00H=Disable, 01H=Enable)		
#define C32Ident		0xC5	//53H	// Code32    ID code		
#define C32InsertCode	0xC6	//00H	// Code32    insert G1 or G2 code  ( 00H=Disable, 01H=G1 code, 02H=G2 code)		
#define C32Check		0xC7	//00H	// Code32    Check CS ( 00H=Disable, 01H=Enable)		
#define C32TranCheck	0xC8	//00H	// Code32    transmit CC ( 00H=Disable, 01H=Enable)		
#define C32CutFNum		0xC9	//00H	// Code32    cut front number		
#define C32CutBNum		0xCA	//00H	// Code32    cut back number		
					
#define G1InsertPosit	0xDA	//00H	//  G1插入位置		
#define G1InsertCode	0xDB	//00H	// DAH~E3H   G1插入码		
#define G1InsertCode_L	0xE4	//00H	目前输入字数		
#define G2InsertPosit	0xE5	//00H	//  G2插入位置		
#define G2InsertCode	0xE6	//00H	// E5H~EEH   G2插入码		
#define G2InsertCode_L	0xEF	//00H	目前输入字数	

/**
* @brief uE988 命令定义  decoder->host
*/
typedef struct {
	unsigned short			CmdPos;
	unsigned short			DataLength;
	unsigned char			status;
	unsigned char			*CmdBuffer;
}THJ5000Command;

#define		RES_CHECKFAILURE			1
#define		RES_UNKOWN_MSG				2
#define		RESPONSE_SUCCESS			3
#define		RESPONSE_ACK				4
#define		RESPONSE_NAK				5



//static unsigned char	g_ack_enable;							//indicate whether ack/nck handshaking is enabled  1: enable; 0: disable
#define MAX_DECODE_DATA		50

THJ5000Command	g_resCmd;		//scan decoder -> host
unsigned char	*g_pReqCmd;		//host -> scan decoder
static	unsigned int	wait_time_out;			//get_barcode命令的等待超时设置



#define G_SEND_BUF_LENGTH     32
#define G_RECEIV_BUF_LENGTH   128

unsigned char		g_send_buff[G_SEND_BUF_LENGTH];
unsigned char		g_receive_buff[G_RECEIV_BUF_LENGTH];


static int write_cmd_to_scanner(const unsigned char *pData, unsigned short length);
static int pack_set_command(unsigned char param_offset, unsigned char param_value);
static int pack_ctrl_command(unsigned char cmd_type);
int HJ5000_RxISRHandler(unsigned char c);


/*
 * @brief: 初始化模块端口
*/
static void HJ5000_GPIO_config(void)
{
	GPIO_InitTypeDef				GPIO_InitStructure;
	USART_InitTypeDef				USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
		RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);


	/* PWRDWN PA.5 WAKE	PA.6 TRIG PA.7 */
	//GPIO_InitStructure.GPIO_Pin				= GPIO_Pin_5;
	//GPIO_InitStructure.GPIO_Speed			= GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_Mode			= GPIO_Mode_IN_FLOATING;
	//GPIO_Init(GPIOA, &GPIO_InitStructure);

	//GPIO_InitStructure.GPIO_Pin				= GPIO_Pin_6 | GPIO_Pin_7;
	//GPIO_InitStructure.GPIO_Mode			= GPIO_Mode_Out_PP;
	//GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);

	// 使用UART2, PA2,PA3
	/* Configure USART2 Tx (PA.2) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin				= GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed			= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode			= GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART2 Rx (PA.3) as input floating				*/
	GPIO_InitStructure.GPIO_Pin				= GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode			= GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

#if(USART_RX_MODE == USART_RX_DMA_MODE)
	DMA_InitTypeDef DMA_InitStructure;
        NVIC_InitTypeDef NVIC_InitStructure;

	/* DMA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);


	/* fill init structure */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	/* DMA1 Channel2 (triggered by USART3 Tx event) Config */
	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)(&USART3->DR);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	/* As we will set them before DMA actually enabled, the DMA_MemoryBaseAddr
	 * and DMA_BufferSize are meaningless. So just set them to proper values
	 * which could make DMA_Init happy.
	 */
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)0;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);


	//DMA1通道3配置  
	DMA_DeInit(DMA1_Channel3);  
	//外设地址  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DR);  
	//内存地址  
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)g_receive_buff;  
	//dma传输方向单向  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  
	//设置DMA在传输时缓冲区的长度  
	DMA_InitStructure.DMA_BufferSize = G_RECEIV_BUF_LENGTH;  
	//设置DMA的外设递增模式，一个外设  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	//设置DMA的内存递增模式  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	//外设数据字长  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	//内存数据字长  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
	//设置DMA的传输模式  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
	//设置DMA的优先级别  
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
	//设置DMA的2个memory中的变量互相访问  
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
	DMA_Init(DMA1_Channel3,&DMA_InitStructure);  

	//使能通道3 
	DMA_Cmd(DMA1_Channel3,ENABLE);  

#endif

	//初始化参数    
	//USART_InitStructure.USART_BaudRate = DEFAULT_BAUD;    
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;    
	USART_InitStructure.USART_StopBits = USART_StopBits_1;    
	USART_InitStructure.USART_Parity = USART_Parity_No;    
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;    
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      
	USART_InitStructure.USART_BaudRate = 9600;   
	//初始化串口   
	USART_Init(USART2,&USART_InitStructure); 


#if(USART_RX_MODE == USART_RX_DMA_MODE)
	//中断配置  
	USART_ITConfig(USART3,USART_IT_TC,DISABLE);  
	USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);  
	USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);    

	//配置UART3中断  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;               //通道设置为串口1中断    
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //中断占先等级0    
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;              //中断响应优先级0    
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //打开中断    
	NVIC_Init(&NVIC_InitStructure);  

	/* Enable the DMA1 Channel2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ClearFlag(DMA1_FLAG_TC2);

	//采用DMA方式接收  
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE); 

	/* Enable USART3 DMA Tx request */
	USART_DMACmd(USART3, USART_DMAReq_Tx , ENABLE);

#endif
	//启动串口    
	USART_Cmd(USART2, ENABLE);
}

static void HJ5000_NVIC_config(void)
{
#if(USART_RX_MODE == USART_RX_ISR_MODE)
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel				=USART2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority	= 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ClearITPendingBit(USART2, USART_IT_RXNE); 
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
#endif
}

static void reset_resVar(void)
{
	g_resCmd.CmdPos = 0;
	g_resCmd.DataLength = 0;
	g_resCmd.status	 = 0;
}

/**
* @brief  发数据给条形码扫描仪
* @param[in] unsigned char *pData 要发送的数据
* @param[in] int length 要发送数据的长度
*/
static void send_data_to_scanner(const unsigned char *pData, unsigned short length)
{
#if (USART_RX_MODE == USART_RX_ISR_MODE)
	while(length--)
	{
		USART_SendData(USART2, *pData++);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		{
		}
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){};
#else
	/* disable DMA */
	DMA_Cmd(DMA1_Channel2, DISABLE);

	/* set buffer address */
	memcpy(g_send_buff,pData,length);

	DMA1_Channel2->CMAR = (u32)&g_send_buff[0];
	/* set size */
	DMA1_Channel2->CNDTR = length;

	USART_DMACmd(USART3, USART_DMAReq_Tx , ENABLE);
	/* enable DMA */
	DMA_Cmd(DMA1_Channel2, ENABLE);

	 while(DMA1_Channel2->CNDTR);
         while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET){};
#endif
}

/**
* @brief  发命令给条形码扫描仪
* @param[in] unsigned char *pData 要发送的数据
* @param[in] int length 要发送数据的长度
* @param[out]	0: 成功
*				-1: 失败
*/
static int write_cmd_to_scanner(const unsigned char *pData, unsigned short length)
{

	//HJ5000_wakeup();		//先唤醒模块

	send_data_to_scanner(pData, length);

	//if (g_ack_enable == 0)	//ack/nak handshaking disabled
	//{
	//	return 0;
	//}
	
	return 0;
}


/**
* @brief  打包命令(host->scanner)
* @param[in] unsigned char cmd_code 命令代码
* @param[in] unsigned char cmd_status 
* @param[in] unsigned char *pCmddata 命令数据
* @param[in] unsigned char data_len 命令数据长度
* @note: 共4种命令格式（都是9Bytes）
*     1. 设置命令:0xff 0x55 0x55 0x98 (参数位置)(1byte) (参数值)(1byte) 0x11 0x22 0x33
*	  2. Trig命令：0xff 0x55 0x55 0xaf 0x11 0x22 0x33 0x44 0x55
*	  3. Read CFG命令：0xff 0x55 0x55 0x92 0x11 0x22 0x33 0x44 0x55
*     4. Set ALL Default命令：0xff 0x55 0x55 0xa1 0x11 0x22 0x33 0x44 0x55 		//reponse : 0x79 0x00 0x11 0x22 0x33 0x44
*/
static int pack_set_command(unsigned char param_offset, unsigned char param_value)
{       
    memset(g_pReqCmd,0,10);

	g_pReqCmd[0] = 0xff;
	g_pReqCmd[1] = 0x55;	
	g_pReqCmd[2] = 0x55;	
	g_pReqCmd[3] = 0x98;
	g_pReqCmd[4] = param_offset;
	g_pReqCmd[5] = param_value;
	g_pReqCmd[6] = 0x11;
	g_pReqCmd[7] = 0x22;
	g_pReqCmd[8] = 0x33;
	
	return 9;
}

static int pack_ctrl_command(unsigned char cmd_type)
{       
    memset(g_pReqCmd,0,10);

	g_pReqCmd[0] = 0xff;
	g_pReqCmd[1] = 0x55;	
	g_pReqCmd[2] = 0x55;
	switch(cmd_type)
	{
		case HJ5000_START_DECODE:
			g_pReqCmd[3] = 0xaf;
			break;
		case HJ5000_READ_CFG:
			g_pReqCmd[3] = 0x92;
			break;
		case HJ5000_SET_ALL_DEFAULT:
			g_pReqCmd[3] = 0xA1;
			break;
		default:
			return 0;
	}
	g_pReqCmd[4] = 0x11;
	g_pReqCmd[5] = 0x22;
	g_pReqCmd[6] = 0x33;
	g_pReqCmd[7] = 0x44;
	g_pReqCmd[8] = 0x55;
	
	return 9;
}


void scanner_mod_reset(void)
{
	//GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	g_resCmd.CmdPos = 0;
	g_resCmd.DataLength = 0;
	g_resCmd.status	 = 0;
}

/*
* @brief: 模块初始化
*/
void scanner_mod_init(void)
{
	int	ret;
	HJ5000_GPIO_config();
	HJ5000_NVIC_config();
	
	g_pReqCmd	= g_send_buff;
	g_resCmd.CmdBuffer	= g_receive_buff;

	reset_resVar();
	//初始化串口配置
	//Comm_SetReceiveProc(COMM2, (CommIsrInByte)HJ5000_RxISRHandler);						//设置串口回调函数

	ret = pack_set_command(ScanMode,0x0a);
	write_cmd_to_scanner(g_pReqCmd, ret);
	OSTimeDlyHMSM(0, 0, 0, 100);
	ret = pack_set_command(RSTerminator,0x02);
	write_cmd_to_scanner(g_pReqCmd, ret);
	OSTimeDlyHMSM(0, 0, 0, 100);
	wait_time_out = 28;
	//scan_start = 0;
}


/**
* @brief 处理host收到scanner的数据
* @param[in] unsigned char c 读入的字符
* @return 0:success put in buffer
*        -1:fail
*/
int HJ5000_RxISRHandler(unsigned char c)
{
	//unsigned short checksum = 0;

	if(g_resCmd.status == 0)
	{
		g_resCmd.CmdBuffer[g_resCmd.CmdPos++] = c;
	}
	

	//if (g_resCmd.CmdPos == 1)
	//{
	//	if (g_resCmd.CmdBuffer[0] == 0x06)
	//	{
	//		g_resCmd.status = RESPONSE_ACK;
	//	}
	//	else if (g_resCmd.CmdBuffer[0] == 0x15)
	//	{
	//		g_resCmd.status = RESPONSE_NAK;
	//	}
	//}

	//if ((g_resCmd.CmdBuffer[g_resCmd.CmdPos-1] == 0x0a)&&(g_resCmd.CmdBuffer[g_resCmd.CmdPos-2] == 0x0d))
	if (g_resCmd.CmdBuffer[g_resCmd.CmdPos-1] == 0x0d)
	{
		g_resCmd.status = RESPONSE_SUCCESS;
	}

	if (g_resCmd.CmdPos >= G_RECEIV_BUF_LENGTH)
	{
		reset_resVar();
		g_resCmd.status = RES_UNKOWN_MSG;
		return -1;
	}

	return 0;
}


/*
* @breif:  开始 或 停止扫描条码
* @param[in]: ctrl_type: UE988_START_DECODE  UE988_STOP_DECODE
*/
void HJ5000_start_stop_decode(unsigned char ctrl_type)
{
	int ret;
	if (ctrl_type == HJ5000_START_DECODE)
	{
		ret = pack_ctrl_command(HJ5000_START_DECODE);
	}
	else
	{
		ret = pack_ctrl_command(HJ5000_STOP_DECODE);
	}
	if(ret)
	{
		//g_ack_enable = 0;
		ret = write_cmd_to_scanner(g_pReqCmd, ret);
		//g_ack_enable = 1;
	}

	return;
}

/*
* @breif:  获取条形码
* @param[out]: unsigned char *code_type: 条形码类型		10个字节
* @param[out]: unsigned char *code_buf: 存储条形码的缓存, code Type + decode data
* @param[in]:  unsigned char inbuf_size: 传进来用来存放decode_data的buf大小
* @param[out]  unsigned char *code_len:	 实际获取的条形码的长度，如果实际获取的长度比传进来的buf大，那么只返回传进来的buf大小的数据
*/
int scanner_get_barcode(unsigned char *barcode,unsigned int max_num,unsigned char *barcode_type,unsigned int *barcode_len)
{
	int		i = 0;
#if 0
	//start decode
	HJ5000_start_stop_decode(HJ5000_START_DECODE);
	reset_resVar();
    for(i = 0; i < wait_time_out; i++)   //新扫描头的超时只有3S左右
	{
		if (g_resCmd.status == RESPONSE_SUCCESS) //成功收到响应
		{
			HJ5000_start_stop_decode(HJ5000_STOP_DECODE);
			*barcode_len	= g_resCmd.CmdPos-1;
			memcpy(barcode, &g_resCmd.CmdBuffer[0], ((*barcode_len > max_num)?max_num:*barcode_len));
			strcpy((char*)barcode_type, "");
			//Beep(400);
			return 0;
		}//成功收到响应
		//else if (g_resCmd.status == RESPONSE_NAK)	//响应失败
		//{
		//	lineccd_start_stop_decode(LINECCD_STOP_DECODE);
		//	return -1;
		//}
		//else if (g_resCmd.status == RES_CHECKFAILURE)
		//{
		//	send_nak_to_sanner(RES_CHECKFAILURE);	
		//	lineccd_start_stop_decode(LINECCD_STOP_DECODE);
		//	return -1;
		//}
		else if (g_resCmd.status == RES_UNKOWN_MSG)
		{
			HJ5000_start_stop_decode(HJ5000_STOP_DECODE);
			return -1;
		}

		OSTimeDlyHMSM(0, 0, 0, 100);
	}//延时

	HJ5000_start_stop_decode(HJ5000_STOP_DECODE);
	return -1;
#endif
        
        *barcode_len	= 13;
			memcpy(barcode,"1234567890123" ,13);
			strcpy((char*)barcode_type, "");
        OSTimeDlyHMSM(0, 0, 0, 500);
        return 0;
}


void scanner_power_off(void)
{
	//GPIO_SetBits(SCANNER_POWER_EN_GPIO_PORT,SCANNER_POWER_EN_PIN);
}