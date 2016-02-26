#ifndef _WBTDS01_H_
#define _WBTDS01_H_

#define		WBTD_RES_BUFFER_LEN			128		//从WBTDS01的数据手册上看，没有超过128字节的响应数据

extern unsigned char	wbtd_recbuffer[WBTD_RES_BUFFER_LEN];
//定义蓝牙模块支持的波特率
//@note  未经验证，模块资料也没有给出其支持的波特率列表
typedef enum
{
	BAUDRATE_9600,
	BAUDRATE_19200,
	BAUDRATE_38400,
	BAUDRATE_43000,
	BAUDRATE_57600,
	BAUDRATE_115200
}WBTD_BAUDRATE;


//定义蓝牙模块支持的profile
typedef enum
{
	BT_PROFILE_HID,
	BT_PROFILE_SPP,
	BT_PROFILE_BLE
}BT_PROFILE;

#define BT_MODULE_STATUS_CONNECTED		1
#define BT_MODULE_STATUS_DISCONNECT		2

void WBTD_Reset(void);
int WBTD_init(void);
int WBTD_query_version(unsigned char *ver_buffer);
int WBTD_set_name(unsigned char *name);
int WBTD_set_baudrate(WBTD_BAUDRATE baudrate);
int WBTD_set_autocon(unsigned char mode);
int WBTD_set_profile(BT_PROFILE mode);
int WBTD_set_ioskeypad(unsigned char enable);
int WBTD_hid_send(unsigned char *str,unsigned int len,unsigned int *send_len);
int WBTD_got_notify_type(void);
int WBTD_RxISRHandler(unsigned char *res, unsigned int res_len);

int WBTD_hid_send_test(void);		//for test
#endif