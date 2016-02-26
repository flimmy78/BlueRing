/**
*  @file 	Terminal_Para.h
*  @brief  	定义了终端需要保存在NVM区域的参数的数据结构
*  @note	
*/

#ifndef _TERMINAL_PARA_H_
#define _TERMINAL_PARA_H_

//定义键盘的语言类型
#define KB_LAN_US		1		//英语
#define KB_LAN_FR		2		//法语
#define KB_LAN_IT		3		//意大利
#define KB_LAN_GE		4		//德语

//定义休眠超时
#define TIME_OUT_30S_UNIT	120		//此数据待测试

#define TIMEOUT_NEVER		0
#define TIMEOUT_30S			(TIME_OUT_30S_UNIT*1)
#define TIMEOUT_60S			(TIME_OUT_30S_UNIT*2)
#define TIMEOUT_2MIN		(TIME_OUT_30S_UNIT*4)
#define TIMEOUT_5MIN		(TIME_OUT_30S_UNIT*10)
#define TIMEOUT_10MIN		(TIME_OUT_30S_UNIT*20)
#define TIMEOUT_20MIN		(TIME_OUT_30S_UNIT*40)
#define TIMEOUT_1HOUR		(TIME_OUT_30S_UNIT*120)
#define TIMEOUT_2HOUR		(TIME_OUT_30S_UNIT*240)

//定义蓝牙的工作模式
#define BT_MODE_HID			1
#define BT_MODE_SPP			2

//定义机器的工作模式
#define DEVICE_MODE_BT			1		//蓝牙模式
#define DEVICE_MODE_MEM			2		//脱机存储模式
#define DEVICE_MODE_SWITCH		3		//自动切换模式

/**
*@brief 定义存储在SPI FLASH内部终端参数的结构类型
*@ note	
*/
#pragma pack(1)

typedef struct  {
	unsigned int			checkvalue;					//4字节		0	B	此份结构的校验值 crc32			
	unsigned char			struct_ver;					//1字节		4	B	参数版本号，标识此结构的版本
	unsigned char			ios_softkeypad_enable;		//1字节		5	B	是否使能IOS系统的软键盘
	unsigned char			kb_language;				//1字节		6	B	键盘语言
	unsigned char			bt_mode;					//1字节		7	B	BT模块的工作模式
	unsigned char			lower_power_timeout;		//1字节		8	B	进入低功耗模式的超时时间
	unsigned char			auto_con_last_host;			//1字节		9	B	开机是否需要重连上次的主机
	unsigned char			device_state;				//1字节		10  B	终端的工作模式
	unsigned char			rfu[19];					//19字节	11	B	RFU
	unsigned char			endtag[2];					//0x55,0xAA  30      一共32字节
} TTerminalPara;
#pragma pack()

extern TTerminalPara		g_param;				//终端参数

int ReadTerminalPara(void);
int SaveTerminalPara(void);
int DefaultTerminalPara(void);
#endif
