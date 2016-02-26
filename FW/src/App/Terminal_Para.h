/**
*  @file 	Terminal_Para.h
*  @brief  	�������ն���Ҫ������NVM����Ĳ��������ݽṹ
*  @note	
*/

#ifndef _TERMINAL_PARA_H_
#define _TERMINAL_PARA_H_

//������̵���������
#define KB_LAN_US		1		//Ӣ��
#define KB_LAN_FR		2		//����
#define KB_LAN_IT		3		//�����
#define KB_LAN_GE		4		//����

//�������߳�ʱ
#define TIME_OUT_30S_UNIT	120		//�����ݴ�����

#define TIMEOUT_NEVER		0
#define TIMEOUT_30S			(TIME_OUT_30S_UNIT*1)
#define TIMEOUT_60S			(TIME_OUT_30S_UNIT*2)
#define TIMEOUT_2MIN		(TIME_OUT_30S_UNIT*4)
#define TIMEOUT_5MIN		(TIME_OUT_30S_UNIT*10)
#define TIMEOUT_10MIN		(TIME_OUT_30S_UNIT*20)
#define TIMEOUT_20MIN		(TIME_OUT_30S_UNIT*40)
#define TIMEOUT_1HOUR		(TIME_OUT_30S_UNIT*120)
#define TIMEOUT_2HOUR		(TIME_OUT_30S_UNIT*240)

//���������Ĺ���ģʽ
#define BT_MODE_HID			1
#define BT_MODE_SPP			2

//��������Ĺ���ģʽ
#define DEVICE_MODE_BT			1		//����ģʽ
#define DEVICE_MODE_MEM			2		//�ѻ��洢ģʽ
#define DEVICE_MODE_SWITCH		3		//�Զ��л�ģʽ

/**
*@brief ����洢��SPI FLASH�ڲ��ն˲����Ľṹ����
*@ note	
*/
#pragma pack(1)

typedef struct  {
	unsigned int			checkvalue;					//4�ֽ�		0	B	�˷ݽṹ��У��ֵ crc32			
	unsigned char			struct_ver;					//1�ֽ�		4	B	�����汾�ţ���ʶ�˽ṹ�İ汾
	unsigned char			ios_softkeypad_enable;		//1�ֽ�		5	B	�Ƿ�ʹ��IOSϵͳ�������
	unsigned char			kb_language;				//1�ֽ�		6	B	��������
	unsigned char			bt_mode;					//1�ֽ�		7	B	BTģ��Ĺ���ģʽ
	unsigned char			lower_power_timeout;		//1�ֽ�		8	B	����͹���ģʽ�ĳ�ʱʱ��
	unsigned char			auto_con_last_host;			//1�ֽ�		9	B	�����Ƿ���Ҫ�����ϴε�����
	unsigned char			device_state;				//1�ֽ�		10  B	�ն˵Ĺ���ģʽ
	unsigned char			rfu[19];					//19�ֽ�	11	B	RFU
	unsigned char			endtag[2];					//0x55,0xAA  30      һ��32�ֽ�
} TTerminalPara;
#pragma pack()

extern TTerminalPara		g_param;				//�ն˲���

int ReadTerminalPara(void);
int SaveTerminalPara(void);
int DefaultTerminalPara(void);
#endif
