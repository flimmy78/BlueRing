#ifndef _APP_H_
#define _APP_H_

//主状态机定义：
#define STATE_BT_Mode_Disconnect			1		//蓝牙模式未连接状态
#define STATE_BT_Mode_Connect				2		//蓝牙模式连接状态
#define STATE_BT_Mode_WaitPair				3		//蓝牙模式待配对状态
#define STATE_Memory_Mode					4		//脱机存储模式，蓝牙未连接
#define STATE_Memory_Mode_BT_Connect		5		//脱机存储模式下蓝牙已连接
#define STATE_Memory_Mode_BT_WaitPair		6		//脱机存储模式蓝牙配对状态
//#define STATE_HID_Mode						5		//HID键盘模式
//#define STATE_Auto_Switch_Mode						5		//蓝牙与脱机存储自由切换模式


//事件列表：
//按键事件
#define EVENT_SCAN_KEY_SINGLE_CLICK				1
#define EVENT_SCAN_KEY_DOUBLE_CLICK				(EVENT_SCAN_KEY_SINGLE_CLICK+1)
#define EVENT_SCAN_KEY_LONG_PRESS				(EVENT_SCAN_KEY_DOUBLE_CLICK+1)
//#define EVENT_ERASE_KEY_SINGLE_CLICK			(EVENT_SCAN_KEY_LONG_PRESS+1)
//#define EVENT_ERASE_KEY_LONG_PRESS				(EVENT_ERASE_KEY_SINGLE_CLICK+1)
//#define EVENT_RESET_KEY_PRESS					(EVENT_ERASE_KEY_LONG_PRESS+1)

//USB线的插入拔出
//#define EVENT_USB_CABLE_INSERT					(EVENT_RESET_KEY_PRESS+1)
#define EVENT_USB_CABLE_INSERT					(EVENT_SCAN_KEY_LONG_PRESS+1)
#define EVENT_USB_CABLE_REMOVE					(EVENT_USB_CABLE_INSERT+1)

//蓝牙连接状态的变化
#define EVENT_BT_CONNECTED						(EVENT_USB_CABLE_REMOVE+1)
#define EVENT_BT_DISCONNECTED					(EVENT_BT_CONNECTED+1)

//低电压
#define EVENT_LOW_POWER							(EVENT_BT_DISCONNECTED+1)

//串口中断获取到扫描条码，由于扫描引擎利用硬件IO直接驱动，串口中断接收返回条码，所以扫描引擎获取到条码也当作一个异步事件
#define EVENT_SCAN_GOT_BARCODE							(EVENT_LOW_POWER+1)
//#define EVENT_ERASE_GOT_BARCODE							(EVENT_SCAN_GOT_BARCODE+1)

void app_startup(void);

#endif
