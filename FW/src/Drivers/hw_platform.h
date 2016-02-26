#ifndef _HW_PLATFORM_H_
#define _HW_PLATFORM_H_

//蓝牙模块类型
#define		USE_WBTDS01		1
#define		USE_BT816		2

//定义使用的蓝牙模块
#define BT_MODULE		USE_BT816

#include "keypad.h"
#include "spi_flash.h"
#include "HJ5000_scanner.h"
#include "data_uart.h"
#if(BT_MODULE == USE_WBTDS01)
#include "WBTDS01.h"
#else
#include "BT816.h"
#endif


#define		LED_RED			0
#define		LED_GREEN		1
#define		LED_BLUE		2
//#define		LED_BLUE		3


extern unsigned int	charge_detect_io_cnt;

void hw_platform_init(void);
unsigned int hw_platform_get_PowerClass(void);
unsigned int hw_platform_ChargeState_Detect(void);
unsigned int hw_platform_USBcable_Insert_Detect(void);
void hw_platform_led_ctrl(unsigned int led,unsigned int ctrl);
//void hw_platform_motor_ctrl(unsigned short delay);
void hw_platform_trig_ctrl(unsigned short delay);
void hw_platform_beep_ctrl(unsigned short delay,unsigned int beep_freq);

void hw_platform_start_led_blink(unsigned int led,unsigned short delay);
void hw_platform_stop_led_blink(unsigned int led);
//void hw_platform_beep_motor_ctrl(unsigned short delay,unsigned int beep_freq);

void hw_platform_trip_io(void);

void hw_platform_led_blink_test(void);
#endif
