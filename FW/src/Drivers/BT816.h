#ifndef _BT816_H_
#define _BT816_H_

#define		BT816_RES_BUFFER_LEN			256		//从BT816S01的数据手册上看，没有超过256字节的响应数据

#define SPP_BUFFER_LEN		256

//#define HID_MODE
//#define SPP_MODE


extern unsigned char	BT816_recbuffer[BT816_RES_BUFFER_LEN];
//#ifdef SPP_MODE
extern unsigned char	spp_rec_buffer[SPP_BUFFER_LEN];
extern unsigned int	spp_buffer_head;
extern unsigned int	spp_buffer_tail;
//#endif

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
}BT816_BAUDRATE;


//定义蓝牙模块支持的profile
typedef enum
{
	BT_PROFILE_HID,
	BT_PROFILE_SPP,
	BT_PROFILE_BLE
}BT_PROFILE;

//定义蓝牙模块传输模式
typedef enum
{
	BT_TRANS_MODE_TRANSP,		//透传
	BT_TRANS_MODE_AT			//命令
}BT_TRANS_MODE;

#define BT_MODULE_STATUS_CONNECTED		1
#define BT_MODULE_STATUS_DISCONNECT		2

int BT816_Reset(void);
int BT816_init(void);
int BT816_query_version(unsigned char *ver_buffer);
int BT816_query_name(unsigned char *name);
int BT816_set_name(unsigned char *name);
int BT816_set_baudrate(BT816_BAUDRATE baudrate);
int BT816_enter_pair_mode(void);
int BT816_set_trans_mode(BT_TRANS_MODE mode);
int BT816_set_profile(BT_PROFILE mode);
int BT816_connect_status(void);
int BT816_spp_status(void);
int BT816_hid_set_delay(unsigned int	delay);
int BT816_connect_last_host(void);
int BT816_disconnect(void);
int BT816_toggle_ioskeypad(void);
int BT816_send(unsigned char *str,unsigned int len);
int BT816_set_autocon(unsigned int	enable);

int BT816_RxISRHandler(unsigned char *res, unsigned int res_len);
void BT816_enter_sleep(void);
void BT816_wakeup(void);

int BT816_send_test(void);		//for test
#endif