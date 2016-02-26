/**
* @file app.c
* @brief H520B �����������ݲɼ�����ĿAPP
* @version V0.0.1
* @author joe.zhou
* @date 2015��08��31��
* @note
* @copy
*
* �˴���Ϊ���ںϽܵ������޹�˾��Ŀ���룬�κ��˼���˾δ����ɲ��ø��ƴ�����������
* ����˾�������Ŀ����˾����һ��׷��Ȩ����
*
* <h1><center>&copy; COPYRIGHT 2015 heroje</center></h1>
*
*/
#include "ucos_ii.h"
#include "app.h"
#include "hw_platform.h"
#include "usb_pwr.h"
#include <string.h>
#include <assert.h>
#include "JMemory.h"
#include "TimeBase.h"
#include "hw_config.h"
#include "basic_fun.h"
#include <stdio.h>
#include "record_m.h"
#include "Terminal_Para.h"
#include "usb_lib.h"
#include "PCUsart.h"

//������״̬���߳�������ģ���߳�֮��ͨѶ��IPC����
#define BARCODE_CASH_NUM	15			//����ȴ�����ģ���̷߳��͵����뻺����Ϊ15������
#define MAX_BARCODE_LEN		80			//����������󳤶�Ϊ80���ֽ�

//define the stack size of each task
#define STACK_SIZE_TASKEC			128	
#define STACK_SIZE_TASKSM			356
#define STACK_SIZE_TASKBT			128
#define STACK_SIZE_TASKINI			224

static OS_STK	thread_eventcapture_stk[STACK_SIZE_TASKEC];		//the stack of the Event_capture_thread
static OS_STK	thread_statemachine_stk[STACK_SIZE_TASKSM];		//the stack of the State_Machine_thread
static OS_STK	thread_bt_stk[STACK_SIZE_TASKBT];				//the stack of the BT_Daemon_thread
static OS_STK	*p_init_thread_stk;								//���̵߳�ջ��̬�������߳̽����Լ��ͷ�

static void *barcode_pool[BARCODE_CASH_NUM];	//�����ȡ���������ַ����ĵ�ַ����
static unsigned char barcode_cash[BARCODE_CASH_NUM][MAX_BARCODE_LEN+2];	//���һ���ֽڱ�ʾ���������Ƿ�����pool�д�������
static	unsigned char	lowpower_state;
static	unsigned char	lowpower_cnt;
static  TBATCH_NODE		batch_node;
/*
------------------------------------------------------------
|               barcode[MAX_BARCODE_LEN+1]              |flag|
------------------------------------------------------------
|               barcode[MAX_BARCODE_LEN+1]              |flag|
------------------------------------------------------------
|               barcode[MAX_BARCODE_LEN+1]              |flag|
------------------------------------------------------------
.........
------------------------------------------------------------
|               barcode[MAX_BARCODE_LEN+1]              |flag|
------------------------------------------------------------
*/
//��������ľ�̬�����������⶯̬�����ڴ�
static OS_EVENT	*pBarcode_Queue;			//barcode��Ϣ����


#define EVENT_CASH_NUM		8			//�����¼��Ļ�������
//�����¼�����߳�����״̬��֮��ͨѶ��IPC����
static void *event_pool[EVENT_CASH_NUM];		//�¼�����
OS_EVENT	*pEvent_Queue;			//�¼���Ϣ����

static OS_EVENT *pIOSem;				//IO�ź���
//

unsigned int	device_current_state;		//�豸��״̬��

unsigned int	keypress_timeout;
unsigned char	barcode[MAX_BARCODE_LEN+1];

void u_disk_proc(void);
int lowpower_tip(void);
void system_err_tip(void);

extern void EnterLowPowerMode(void);
extern void ExitLowPowerMode(void);


/**
* @brief	ɨ������ɹ�����ʾ
*/
static inline void scan_barcode_ok_tip(void)
{
	//hw_platform_led_ctrl(LED_BLUE,1);
	hw_platform_beep_ctrl(250,3000);
	
	OSTimeDlyHMSM(0,0,0,10);
	//hw_platform_led_ctrl(LED_BLUE,0);
}
/**
* @brief	����ʧ�ܵ���ʾ
*/
static inline void trans_fail_tip(void)
{
	//hw_platform_led_ctrl(LED_BLUE,1);
	OSTimeDlyHMSM(0,0,0,200);
	hw_platform_beep_ctrl(250,3000);

	OSTimeDlyHMSM(0,0,0,50);
	hw_platform_beep_ctrl(250,3000);
	//hw_platform_led_ctrl(LED_BLUE,0);
	OSTimeDlyHMSM(0,0,0,50);
	hw_platform_beep_ctrl(250,3000);
}
/**
* @brief	��������������ľ�̬�����������ر���ĵ�ַ
* @param[in] unsigned char* barcode				��Ҫ���������
* @return   ����ĵ�ַ
* @note ����:ֻҪ�ҵ�һ����λ�þͷŽ�ȥ��ÿһ�е����һ���ֽ�Ϊ0��ʾ��λ���ǿյ�
*											     ���һ���ֽ�0x55��ʾ��λ���Ѿ�����������
*/
unsigned char * push_barcode_into_cash(unsigned char* barcode)
{
	unsigned int	i;
	for (i = 0; i < BARCODE_CASH_NUM;i++)
	{
		if (barcode_cash[i][MAX_BARCODE_LEN+1] == 0)
		{
			if (strlen((char const*)barcode) > MAX_BARCODE_LEN)
			{
				memcpy(barcode_cash[i],barcode,MAX_BARCODE_LEN);
				barcode_cash[i][MAX_BARCODE_LEN] = 0;
			}
			else
			{
				strcpy((char*)barcode_cash[i],(char const*)barcode);
			}
			barcode_cash[i][MAX_BARCODE_LEN+1] = 0x55;		//��ʾ�Ѿ�����������
			return barcode_cash[i];
		}
	}

	return (unsigned char*)0;
}


/**
* @brief	���Ѿ����ͳ�ȥ�Ļ����ַpull����
* @param[in] unsigned char* barcode_addr    һ���Ϸ��Ļ������ĵ�ַ
* @return   none
*/
void pull_barcode_from_cash(unsigned char* barcode_addr)
{
	assert((int)barcode_addr >= (int)barcode_cash[0]);
	assert((int)barcode_addr <= (int)barcode_cash[BARCODE_CASH_NUM-1]);
	assert(((int)barcode_addr - (int)barcode_cash[0])%(MAX_BARCODE_LEN+2) == 0);

	//memset(barcode_addr,0,MAX_BARCODE_LEN+2);
	barcode_addr[MAX_BARCODE_LEN+1] = 0;		//�ѱ�־�ָ�Ϊ0����
	return;
}



/**
* @brief	����Memoryģʽʱ����Ҫ���е�һЩ����
*/
static inline void enter_into_Memory_Mode(void)
{
#ifdef DEBUG_VER
	printf("enter into Memory Mode\r\n");
#endif
#if(BT_MODULE == USE_BT816)
	//BT816_enter_sleep();
#endif
	hw_platform_led_ctrl(LED_GREEN,1);
	g_param.device_state = DEVICE_MODE_MEM;
	SaveTerminalPara();
}

/**
* @brief	�˳�Memoryģʽʱ����Ҫ���е�һЩ����
*/
static inline void exit_from_Memory_Mode(void)
{
#ifdef DEBUG_VER
	printf("exit from Memory Mode\r\n");
#endif
	//@todo...
	hw_platform_led_ctrl(LED_GREEN,0);
}

/**
* @brief	����USB HIDģʽʱ����Ҫ���е�һЩ����
*/
//static inline void enter_into_USB_HID_Mode(void)
//{
//#ifdef DEBUG_VER
//	printf("enter into USB HID Mode\r\n");
//#endif
//#if(BT_MODULE == USE_BT816)
//	BT816_enter_sleep();
//#endif
//	hw_platform_led_ctrl(LED_RED,1);
//	//hw_platform_beep_ctrl(100,1045);
//	//hw_platform_beep_ctrl(100,1171);
//	//hw_platform_beep_ctrl(100,1316);
//	//hw_platform_beep_ctrl(100,1393);
//	//hw_platform_beep_ctrl(100,1563);
//	//hw_platform_beep_ctrl(100,1755);
//	//hw_platform_beep_ctrl(100,1971);
//
//	hw_platform_beep_ctrl(100,1316);
//	hw_platform_beep_ctrl(100,1316);
//	hw_platform_beep_ctrl(100,1393);
//	hw_platform_beep_ctrl(100,1563);
//	hw_platform_beep_ctrl(100,1563);
//	hw_platform_beep_ctrl(100,1393);
//	hw_platform_beep_ctrl(100,1316);
//	hw_platform_beep_ctrl(100,1171);
//}

///**
//* @brief	�˳�USB HIDģʽʱ����Ҫ���е�һЩ����
//*/
//static inline void exit_from_USB_HID_Mode(void)
//{
//#ifdef DEBUG_VER
//	printf("exit from USB HID Mode\r\n");
//#endif
//	hw_platform_led_ctrl(LED_RED,0);
//}

/**
* @brief	����BTģʽʱ����Ҫ���е�һЩ����
* @param[in] unsigned char	child_state		STATE_BT_Mode_Disconnect  or STATE_BT_Mode_Connect or STATE_BT_Mode_WaitPair
* @param[in] unsigned char	save			0: ����Ҫ���滷������		1:��Ҫ���滷������
*/
static inline void enter_into_BT_Mode(unsigned char child_state,unsigned char save)
{
#ifdef DEBUG_VER
	printf("enter into BT Mode:%d\r\n",child_state);
#endif
#if(BT_MODULE == USE_BT816)
	BT816_wakeup();
#endif
	if (child_state == STATE_BT_Mode_WaitPair)
	{
#if(BT_MODULE == USE_WBTDS01)
		WBTD_Reset();
#else
		BT816_enter_pair_mode();
#endif
		hw_platform_beep_ctrl(300,3000);
		hw_platform_start_led_blink(LED_BLUE,10);
#if(BT_MODULE == USE_WBTDS01)
		WBTD_set_autocon(0);
#endif
	}
	else if (child_state == STATE_BT_Mode_Disconnect)
	{
		hw_platform_beep_ctrl(300,3000);
		hw_platform_start_led_blink(LED_BLUE,150);
	}
	else
	{
		hw_platform_beep_ctrl(300,3000);
		hw_platform_led_ctrl(LED_BLUE,1);
	}

	if (save)
	{
		g_param.device_state = DEVICE_MODE_BT;
		SaveTerminalPara();
	}
}

/**
* @brief	�˳�BTģʽʱ����Ҫ���е�һЩ����
* @param[in] unsigned char	child_state		STATE_BT_Mode_Disconnect  or STATE_BT_Mode_Connect or STATE_BT_Mode_WaitPair
*/
static inline void exit_from_BT_Mode(unsigned char child_state)
{
#ifdef DEBUG_VER
	printf("exit from BT Mode:%d\r\n",child_state);
#endif
	if (child_state == STATE_BT_Mode_Connect)
	{
#if(BT_MODULE == USE_WBTDS01)
		WBTD_set_autocon(1);
#else
		//BT816_set_autocon(0);
		BT816_disconnect();
#endif
		//delay_ms(1);
		hw_platform_beep_ctrl(300,3000);
		hw_platform_led_ctrl(LED_BLUE,0);
#if(BT_MODULE == USE_WBTDS01)
		WBTD_Reset();//�����Ͽ�����������������	
#endif	
	}
	else
	{
		hw_platform_stop_led_blink(LED_BLUE);
	}
}


/**
* @brief	ͨ��USB HID��������
*/
static void barcode_hid_send(unsigned char* barcode)
{
	unsigned int	i,code_len;
	unsigned char key_value_report[8];

    code_len = strlen((char const*)barcode);
	OSSchedLock();
	for (i = 0; i < code_len; i++)
	{
		ascii_to_keyreport(barcode[i],key_value_report);

		SendData_To_PC(key_value_report, 3);
		SendData_To_PC("\x00\x00\x00", 3);
	}

	memcpy(key_value_report,"\x00\x00\x28",3);	//����

	SendData_To_PC(key_value_report, 3);
	SendData_To_PC("\x00\x00\x00", 3);	//����
	OSSchedUnlock();
}

/**
* @brief	Ӧ�õĳ�ʼ��
*/
void app_init(void)
{
	//����һ����Ϣ���У����ڽ��¼������̺߳�����ģ���̻߳�ȡ���첽�¼�֪ͨ����״̬���߳�
	pEvent_Queue = OSQCreate((void**)&event_pool,EVENT_CASH_NUM);
	assert(pEvent_Queue != (OS_EVENT*)0);

	//����һ����Ϣ���У����ڽ���״̬���̻߳�ȡ�����봫�͵�����ģ���߳�.
	pBarcode_Queue =OSQCreate((void**)&barcode_pool,BARCODE_CASH_NUM);
	assert(pBarcode_Queue != (OS_EVENT*)0);
	memset(barcode_cash,0,BARCODE_CASH_NUM*(MAX_BARCODE_LEN+2));

	//����һ���ź���������IO�ж�֪ͨ�¼������̣߳����ⲿIO��������Ҫ�¼������߳̿�ʼ��ȡ�����¼��Ķ���
	pIOSem = OSSemCreate(0);
	assert(pIOSem != (OS_EVENT*)0);

	lowpower_state = 0;
	lowpower_cnt = 0;
}

typedef int(*set_code_proc)(void);		//����ֵΪ1��ʾ��Ҫ���滷������������������Ҫ

typedef struct {
	const unsigned char*	set_code;
	set_code_proc			setCode_proc;	
}SETCODE_HANDLE;

int setcode_KB_EN_proc(void)
{
	g_param.kb_language = KB_LAN_US;
	return 1;
}
int setcode_KB_FR_proc(void)
{
	g_param.kb_language = KB_LAN_FR;
	return 1;
}
int setcode_KB_IT_proc(void)
{
	g_param.kb_language = KB_LAN_IT;
	return 1;
}
int setcode_KB_GE_proc(void)
{
	g_param.kb_language = KB_LAN_GE;
	return 1;
}

int setcode_BT_MODE_proc(void)
{
	if ((device_current_state == STATE_Memory_Mode)||(device_current_state == STATE_Memory_Mode_BT_Connect)||(device_current_state == STATE_Memory_Mode_BT_WaitPair))
	{
		exit_from_Memory_Mode();
		if (device_current_state == STATE_Memory_Mode)
		{
			device_current_state = STATE_BT_Mode_Disconnect;
		}
		else if (device_current_state == STATE_Memory_Mode_BT_Connect)
		{
			device_current_state = STATE_BT_Mode_Connect;
		}
		else
		{
			device_current_state = STATE_BT_Mode_WaitPair;
		}
		
		enter_into_BT_Mode(device_current_state,1);		//��������Ѿ������˲���
	}

	return 0;
}

int setcode_MEM_MODE_proc(void)
{
	if ((device_current_state != STATE_Memory_Mode)&&(device_current_state != STATE_Memory_Mode_BT_Connect)&&(device_current_state != STATE_Memory_Mode_BT_WaitPair))
	{
		//exit_from_BT_Mode(device_current_state);
		if (device_current_state == STATE_BT_Mode_Disconnect)
		{
			device_current_state = STATE_Memory_Mode;
		}
		else if (device_current_state == STATE_BT_Mode_Connect)
		{
			device_current_state = STATE_Memory_Mode_BT_Connect;
		}
		else if (device_current_state == STATE_BT_Mode_WaitPair)
		{
			device_current_state = STATE_Memory_Mode_BT_WaitPair;
		}
		
		enter_into_Memory_Mode();
	}

	return 0;
}

int setcode_SWITCH_MODE_proc(void)
{
	g_param.device_state = DEVICE_MODE_SWITCH;
	if(device_current_state == STATE_Memory_Mode_BT_Connect)
	{
		hw_platform_led_ctrl(LED_GREEN,0);
		device_current_state = STATE_BT_Mode_Connect;
	}
	else if (device_current_state == STATE_BT_Mode_Disconnect)
	{
		//exit_from_Memory_Mode();
		device_current_state = STATE_Memory_Mode;
		hw_platform_led_ctrl(LED_GREEN,1);
	}
	else if (device_current_state == STATE_BT_Mode_WaitPair)
	{
		device_current_state = STATE_Memory_Mode_BT_WaitPair;
		hw_platform_led_ctrl(LED_GREEN,1);
	}
	return 1;
}

int setcode_TM_30s_proc(void)
{
	g_param.lower_power_timeout = TIMEOUT_30S;
	return 1;
}

int setcode_TM_60s_proc(void)
{
	g_param.lower_power_timeout = TIMEOUT_60S;
	return 1;
}

int setcode_TM_2min_proc(void)
{
	g_param.lower_power_timeout = TIMEOUT_2MIN;
	return 1;
}

int setcode_TM_5min_proc(void)
{
	g_param.lower_power_timeout = TIMEOUT_5MIN;
	return 1;
}

int setcode_TM_10min_proc(void)
{
	g_param.lower_power_timeout = TIMEOUT_10MIN;
	return 1;
}

int setcode_TM_20min_proc(void)
{
	g_param.lower_power_timeout = TIMEOUT_20MIN;
	return 1;
}

int setcode_TM_1hour_proc(void)
{
	g_param.lower_power_timeout = TIMEOUT_1HOUR;
	return 1;
}

int setcode_TM_2hour_proc(void)
{
	g_param.lower_power_timeout = TIMEOUT_2HOUR;
	return 1;
}

int setcode_TM_never_proc(void)
{
	g_param.lower_power_timeout = TIMEOUT_NEVER;
	return 1;
}

//�ϴ��ѻ��洢������
int setcode_MEM_UPLOAD_proc(void)
{
	unsigned int	i,cnt;
	unsigned char	*rec,*pCache;
	int				ret;
	if (device_current_state == STATE_Memory_Mode_BT_Connect || device_current_state == STATE_BT_Mode_Connect)
	{
		cnt  = record_module_count();
		if (cnt)
		{
			hw_platform_beep_ctrl(200,4000);
		}
		hw_platform_start_led_blink(LED_GREEN,6);
		for (i = 0; i < cnt;i++)
		{
			rec = get_node((i==0)?0:2,0);
			if (rec)
			{
				//barcode_hid_send(((TBATCH_NODE*)rec)->barcode);
				//if (i%10 == 0)
				//{
				//	hw_platform_led_ctrl(LED_GREEN,(i+1)%2);
					//led_state = ~led_state;
				//}
reCache1:
					pCache = push_barcode_into_cash(((TBATCH_NODE*)rec)->barcode);
					if (pCache == (unsigned char*)0)
					{
						OSTimeDlyHMSM(0,0,0,50);
						goto reCache1;
					}

repost1:
				ret = OSQPost(pBarcode_Queue,(void*)pCache);
				if(ret != OS_ERR_NONE)
				{
					if(ret == OS_ERR_Q_FULL)
					{
						//����������˻��߿��¼�ʱ����ô��Ҫ��ʱ����
						OSTimeDlyHMSM(0,0,0,100);
						goto repost1;
					}
					else
					{
						assert(0);	//ϵͳ����
					}
				}
			}
			else
			{
				//��ʾ�û���ȡʧ��һ��
				//@todo...
			}
		}

		if (cnt)
		{
			hw_platform_beep_ctrl(200,4000);
		}

		hw_platform_stop_led_blink(LED_GREEN);
		if(device_current_state == STATE_Memory_Mode_BT_Connect)
		{
			hw_platform_led_ctrl(LED_GREEN,1);
		}
	}

	
	return 0;
}

//�ϴ��ѻ��洢����������
int setcode_MEM_TOTAL_proc(void)
{
	unsigned int	cnt;
	unsigned char	tmp_str[20];
	int				ret;
	unsigned char	*pCache;
	if (device_current_state == STATE_Memory_Mode_BT_Connect || device_current_state == STATE_BT_Mode_Connect)
	{
		cnt = record_module_count();
		sprintf(tmp_str,"Total:%d",cnt);
reCache2:
		pCache = push_barcode_into_cash(tmp_str);
		if (pCache == (unsigned char*)0)
		{
			OSTimeDlyHMSM(0,0,0,50);
			goto reCache2;
		}
repost2:
		ret = OSQPost(pBarcode_Queue,(void*)pCache);
		if(ret != OS_ERR_NONE)
		{
			if(ret == OS_ERR_Q_FULL)
			{
				//����������˻��߿��¼�ʱ����ô��Ҫ��ʱ����
				OSTimeDlyHMSM(0,0,0,100);
				goto repost2;
			}
			else
			{
				assert(0);	//ϵͳ����
			}
		}

	}
	return 0;
}

//����ѻ��洢������
int setcode_MEM_CLEAR_proc(void)
{
	hw_platform_start_led_blink(LED_GREEN,6);
	record_clear();
	hw_platform_stop_led_blink(LED_GREEN);
	if ((device_current_state == STATE_Memory_Mode)||(device_current_state == STATE_Memory_Mode_BT_Connect)||(device_current_state == STATE_Memory_Mode_BT_WaitPair))
	{
		hw_platform_led_ctrl(LED_GREEN,1);
	}
	return 0;
}

//debounce the ios keypad
int setcode_SHOW_PAD_proc(void)
{
	if (g_param.ios_softkeypad_enable)
	{
#if(BT_MODULE == USE_WBTDS01)
		WBTD_set_ioskeypad(1);
#else
		BT816_toggle_ioskeypad();
#endif
	}
	return 0;
}

int setcode_EN_PAD_proc(void)
{
	g_param.ios_softkeypad_enable ^= 1;
	return 1;
}

int setcode_BT_HID_proc(void)
{
	if (g_param.bt_mode != BT_MODE_HID)
	{
		if (BT816_set_profile(BT_PROFILE_HID))
		{
			OSTimeDlyHMSM(0,0,0,50);
			if (BT816_set_profile(BT_PROFILE_HID))
			{
				//��������ʧ��
				hw_platform_beep_ctrl(200,4000);
				OSTimeDlyHMSM(0,0,0,50);
				return 0;
			}
		}
		g_param.bt_mode = BT_MODE_HID;
	}
	
	return 1;
}

int setcode_BT_SPP_proc(void)
{
	if (g_param.bt_mode != BT_MODE_SPP)
	{
		if (BT816_set_profile(BT_PROFILE_SPP))
		{
			OSTimeDlyHMSM(0,0,0,50);
			if (BT816_set_profile(BT_PROFILE_SPP))
			{
				//��������ʧ��
				hw_platform_beep_ctrl(200,4000);
				OSTimeDlyHMSM(0,0,0,50);
				return 0;
			}
		}
		g_param.bt_mode = BT_MODE_SPP;
	}

	return 1;
}

//SETCODE_HANDLE	setcode_tbl[]={
//	//CODE128 ��������
//	//��������ģʽ
//	{"#%EN%",setcode_KB_EN_proc},		//Ӣ��
//	{"#%FR%",setcode_KB_FR_proc},		//����
//	{"#%IT%",setcode_KB_IT_proc},		//�����
//	{"#%GE%",setcode_KB_GE_proc},		//����
//
//	//���ù���ģʽ
//	{"#%BT_MODE%",setcode_BT_MODE_proc},	//����Ϊ����ģʽ
//	{"#%ME_MODE%",setcode_MEM_MODE_proc},	//����Ϊ�ѻ��洢ģʽ
//	{"#%SW_MODE%",setcode_SWITCH_MODE_proc},	//����Ϊ�������ѻ��洢�Զ��л���ģʽ
//	
//	//�������߳�ʱʱ��
//	{"#%TM_30s%%%",setcode_TM_30s_proc},		//30s
//	{"#%TM_60s%%%",setcode_TM_60s_proc},		//60s
//	{"#%TM_2min%%",setcode_TM_2min_proc},	//2min
//	{"#%TM_5min%%",setcode_TM_5min_proc},	//5min
//	{"#%TM_10min%",setcode_TM_10min_proc},	//10min
//	{"#%TM_20min%",setcode_TM_20min_proc},	//20min
//	{"#%TM_1hour%",setcode_TM_1hour_proc},	//1hour
//	{"#%TM_2hour%",setcode_TM_2hour_proc},	//2hour
//	{"#%TM_never%",setcode_TM_never_proc},	//never
//
//	//�����ѻ�ģʽ�µĹ���
//	{"#%MEM_UPLOAD%",setcode_MEM_UPLOAD_proc},	//�ϴ������ѻ��洢������
//	{"#%MEM_TOTAL%%",setcode_MEM_TOTAL_proc},	//�ϴ��ѻ��洢����������
//	{"#%MEM_CLEAR%%",setcode_MEM_CLEAR_proc},	//������е��ѻ�����
//
//	//����IOS����������
//	{"#%SHOW_PAD%",setcode_SHOW_PAD_proc},		//��ʾ��������IOSС����
//	{"#%EN_PAD%%%",setcode_EN_PAD_proc},			//ʹ��IOSС���̵ĵ�������
//	
//	//���������Ĺ���ģʽ
//	{"#%BT_HID%",setcode_BT_HID_proc},		//��������ģ�鹤����HIDģʽ
//	{"#%BT_SPP%",setcode_BT_SPP_proc},		//��������ģ�鹤����SPPģʽ
//};

SETCODE_HANDLE	setcode_tbl[]={
	//CODE128 ��������
	//��������ģʽ
	{"%%EN",setcode_KB_EN_proc},		//Ӣ��
	{"%%FR",setcode_KB_FR_proc},		//����
	{"#%%IT",setcode_KB_IT_proc},		//�����
	{"%%GE",setcode_KB_GE_proc},		//����

	//���ù���ģʽ
	{"%%ALLPT-SET",setcode_BT_MODE_proc},	//����Ϊ����ģʽ
	{"%%ALLMEM-SET",setcode_MEM_MODE_proc},	//����Ϊ�ѻ��洢ģʽ
	{"%%ALLAEM-SET",setcode_SWITCH_MODE_proc},	//����Ϊ�������ѻ��洢�Զ��л���ģʽ

	//�������߳�ʱʱ��
	{"%%ALLTIM01",setcode_TM_30s_proc},		//30s
	{"%%ALLTIM02",setcode_TM_60s_proc},		//60s
	{"#%TM_2min%",setcode_TM_2min_proc},	//2min
	{"#%TM_5min%",setcode_TM_5min_proc},	//5min
	{"#%TM_10min%",setcode_TM_10min_proc},	//10min
	{"#%TM_20min%",setcode_TM_20min_proc},	//20min
	{"#%TM_1hour%",setcode_TM_1hour_proc},	//1hour
	{"#%TM_2hour%",setcode_TM_2hour_proc},	//2hour
	{"#%TM_never%",setcode_TM_never_proc},	//never

	//�����ѻ�ģʽ�µĹ���
	{"%%ALLMEM-SC",setcode_MEM_UPLOAD_proc},	//�ϴ������ѻ��洢������
	{"%%ALLMEM-ZS",setcode_MEM_TOTAL_proc},	//�ϴ��ѻ��洢����������
	{"%%ALLMEM-QC",setcode_MEM_CLEAR_proc},	//������е��ѻ�����

	//����IOS����������
	{"%%ShowPads",setcode_SHOW_PAD_proc},		//��ʾ��������IOSС����
	{"%%BT_Pads_En",setcode_EN_PAD_proc},			//ʹ��IOSС���̵ĵ�������

	//���������Ĺ���ģʽ
	{"%%BT_HID",setcode_BT_HID_proc},		//��������ģ�鹤����HIDģʽ
	{"%%BT_SPP",setcode_BT_SPP_proc},		//��������ģ�鹤����SPPģʽ
};
//��ȡ��ɨ�������֮���Ԥ������Ҫ��Ϊ�˴�����������
//@����ֵ��1����ʾ���������������룬�Ѿ���������Ҫ�ٽ��к�������
//		   0����ʾ�����������룬��Ҫ�����Ĵ���
static int barcode_pre_handle(unsigned char *barcode)
{
	int i;
	for(i = 0; i < sizeof(setcode_tbl)/sizeof(SETCODE_HANDLE); i++)
	{
		if(strcmp((const char*)barcode,(const char*)(setcode_tbl[i].set_code)) == 0)
		{
			if(setcode_tbl[i].setCode_proc())
			{
				SaveTerminalPara();
			}

			hw_platform_beep_ctrl(200,4000);
			return 1;
		}
	}

	return 0;
}



/**
* @brief	ά����״̬�����߳�
*/
void State_Machine_thread(void *p)
{
	unsigned int	i,cnt,event;
	unsigned char	err;
	unsigned char   codetype[20];
	unsigned int    codelen;
	int				ret,index;
	unsigned int	last_state;
	unsigned char	*rec,*pCache;

	Jfree(p_init_thread_stk);	//�˳���ʼ���߳�ʱ���ͷ��Լ�������ջ
	Keypad_Int_Enable();

	//hw_platform_led_blink_test();		//for test
	//lowpower_tip();					//for test
	//record_m_test();					//for test
	
	while(1)
	{
		event = (unsigned int)OSQPend(pEvent_Queue,25,&err);
		if (event == 0)
		{
			if ((g_param.lower_power_timeout)&&(hw_platform_USBcable_Insert_Detect() == 0))
			{
				keypress_timeout++;
				if (keypress_timeout == g_param.lower_power_timeout)
				{
					hw_platform_beep_ctrl(500,3000);	//����һ�ν�������
#ifndef DEBUG_VER
					EnterLowPowerMode();
					ExitLowPowerMode();
#endif	
					hw_platform_beep_ctrl(200,4000);	//����2���˳�����
					OSTimeDlyHMSM(0,0,0,50);
					hw_platform_beep_ctrl(200,4000);
				}
			}
			continue;
		}
#ifdef DEBUG_VER
		printf("current state:%d\r\n",device_current_state);
		printf("event:%d\r\n",event);
#endif
		if(device_current_state ==  STATE_BT_Mode_Disconnect)
		{
			switch(event)
			{
			//case EVENT_SCAN_KEY_SINGLE_CLICK:
			//case EVENT_SCAN_KEY_DOUBLE_CLICK:
			//	ret = scanner_get_barcode(barcode,MAX_BARCODE_LEN,codetype,&codelen);	//ɨ������
			//	hw_platform_stop_led_blink(LED_GREEN);
			//	if (ret == 0)
			case EVENT_SCAN_GOT_BARCODE:
				scan_barcode_ok_tip();
				if (lowpower_state)
				{
					lowpower_tip();
				}
				barcode_pre_handle(barcode);
				//ֻ��ɨ�赽������ѣ�ʲô������
				trans_fail_tip();
				break;
			case EVENT_SCAN_KEY_LONG_PRESS:
				//�л����������ģʽ
				exit_from_BT_Mode(STATE_BT_Mode_Disconnect);
				device_current_state = STATE_BT_Mode_WaitPair;
				enter_into_BT_Mode(STATE_BT_Mode_WaitPair,0);
				break;
			case EVENT_BT_CONNECTED:
				//�л�����������ģʽ
				exit_from_BT_Mode(STATE_BT_Mode_Disconnect);
				device_current_state = STATE_BT_Mode_Connect;
				enter_into_BT_Mode(STATE_BT_Mode_Connect,0);
				break;
			case EVENT_BT_DISCONNECTED:
				break;
			//case EVENT_USB_CABLE_INSERT:
			//	//�л���USB HIDģʽ
			//	exit_from_BT_Mode(0);
			//	last_state = STATE_BT_Mode_Disconnect;
			//	device_current_state = STATE_HID_Mode;
			//	enter_into_USB_HID_Mode();
			//	break;
			//case EVENT_USB_CABLE_REMOVE:
			//	break;
			case EVENT_LOW_POWER:
				lowpower_tip();
				break;
			default:
				break;
			}
		}
		else if(device_current_state ==  STATE_BT_Mode_Connect)
		{
			switch(event)
			{
			//case EVENT_SCAN_KEY_SINGLE_CLICK:
				//ɨ������
			//	ret = scanner_get_barcode(barcode,MAX_BARCODE_LEN,codetype,&codelen);
			//	hw_platform_stop_led_blink(LED_GREEN);
			//	if (ret != 0)
			//	{
			//		break;
			//	}
			case EVENT_SCAN_GOT_BARCODE:
				scan_barcode_ok_tip();
				if (lowpower_state)
				{
					lowpower_tip();
				}
				if (barcode_pre_handle(barcode))
				{
					break;
				}
				//ɨ�赽������
				//����ȡ����������push��cash����������Ȼ��Post��ϵͳ��
				//Queue��������ģ���̸߳���ȥ���͵�����
reCache:
				pCache = push_barcode_into_cash((unsigned char*)barcode);
				if (pCache == (unsigned char *)0)
				{
					goto reCache;
				}
repost:
				ret = OSQPost(pBarcode_Queue,(void*)pCache);
				if(ret != OS_ERR_NONE)
				{
					if(ret == OS_ERR_Q_FULL)
					{
						//����������˻��߿��¼�ʱ����ô��Ҫ��ʱ����
						OSTimeDlyHMSM(0,0,0,100);
						goto repost;
					}
					else
					{
						hw_platform_beep_ctrl(400,4000);
						OSTimeDlyHMSM(0,0,0,50);
						hw_platform_beep_ctrl(400,4000);
						assert(0);	//ϵͳ����
					}
				}
				break;
			case EVENT_SCAN_KEY_DOUBLE_CLICK:
				if (g_param.ios_softkeypad_enable)
				{
#if(BT_MODULE == USE_WBTDS01)
					WBTD_set_ioskeypad(1);
#else
					BT816_toggle_ioskeypad();
#endif
				}
				break;
			case EVENT_SCAN_KEY_LONG_PRESS:
				//�л����������ģʽ
				exit_from_BT_Mode(STATE_BT_Mode_Connect);
				device_current_state = STATE_BT_Mode_WaitPair;
				enter_into_BT_Mode(STATE_BT_Mode_WaitPair,0);
				break;

			case EVENT_BT_CONNECTED:
				break;
			case EVENT_BT_DISCONNECTED:
				//�л��������Ͽ�״̬
				exit_from_BT_Mode(STATE_BT_Mode_Connect);
				
				if (g_param.device_state == DEVICE_MODE_SWITCH)
				{
					//����豸�����Զ��л���ģʽ����ô�����Ͽ����ͻ��Զ�����Momeryģʽ
					device_current_state = STATE_Memory_Mode;
                    hw_platform_led_ctrl(LED_GREEN,1);
				}
				else if(g_param.device_state == DEVICE_MODE_BT)
				{
					device_current_state = STATE_BT_Mode_Disconnect;
                    enter_into_BT_Mode(STATE_BT_Mode_Disconnect,0);
				}
				else
				{
					//MEM_MODE����ģʽ�����ܳ����������״̬
					assert(0);
				}

				break;
			//case EVENT_USB_CABLE_INSERT:
			//	//�л���USB HIDģʽ
			//	exit_from_BT_Mode(STATE_BT_Mode_Connect);
			//	last_state = STATE_BT_Mode_Disconnect;
			//	device_current_state = STATE_HID_Mode;
			//	enter_into_USB_HID_Mode();
			//	break;
			//case EVENT_USB_CABLE_REMOVE:
			//	break;
			case EVENT_LOW_POWER:
				lowpower_tip();
				break;
			default:
				break;
			}
		}
		else if(device_current_state ==  STATE_BT_Mode_WaitPair)
		{
			switch(event)
			{
			case EVENT_SCAN_KEY_SINGLE_CLICK:
			case EVENT_SCAN_KEY_LONG_PRESS:
				exit_from_BT_Mode(STATE_BT_Mode_WaitPair);
				if (g_param.device_state == DEVICE_MODE_SWITCH)
				{
					device_current_state = STATE_Memory_Mode;
				}
				else
				{
					device_current_state = STATE_BT_Mode_Disconnect;
				}
				
				hw_platform_beep_ctrl(300,3000);
				hw_platform_start_led_blink(LED_BLUE,150);
				break;
			case EVENT_SCAN_GOT_BARCODE:
				
				scan_barcode_ok_tip();
				
				if (lowpower_state)
				{
					lowpower_tip();
				}
				barcode_pre_handle(barcode);
				trans_fail_tip();
				//ֻ��ɨ�赽������ѣ�ʲô������
				break;
			case EVENT_BT_CONNECTED:
				//�л�����������״̬
				exit_from_BT_Mode(STATE_BT_Mode_WaitPair);
				device_current_state = STATE_BT_Mode_Connect;
				enter_into_BT_Mode(STATE_BT_Mode_Connect,0);
				break;
			case EVENT_BT_DISCONNECTED:
				break;
			//case EVENT_USB_CABLE_INSERT:
			//	//�л���USB HIDģʽ
			//	exit_from_BT_Mode(STATE_BT_Mode_WaitPair);
			//	last_state = STATE_BT_Mode_WaitPair;
			//	device_current_state = STATE_HID_Mode;
			//	enter_into_USB_HID_Mode();
			//	break;
			//case EVENT_USB_CABLE_REMOVE:
			//	break;
			case EVENT_LOW_POWER:
				lowpower_tip();
				break;
			default:
				break;
			}
		}
		else if(device_current_state ==  STATE_Memory_Mode)
		{
			switch(event)
			{
			//case EVENT_SCAN_KEY_SINGLE_CLICK:
				//ɨ������
			//	ret = scanner_get_barcode(barcode,MAX_BARCODE_LEN,codetype,&codelen);
			//	hw_platform_stop_led_blink(LED_GREEN);
			//	if (ret != 0)
			//	{
			//		break;
			//	}
			case EVENT_SCAN_GOT_BARCODE:
				scan_barcode_ok_tip();
				if (lowpower_state)
				{
					lowpower_tip();
				}
				if (barcode_pre_handle(barcode))
				{
					break;
				}
				//ɨ�赽������
				//����ȡ�������뱣�浽memory
				memset((void*)&batch_node,0,sizeof(TBATCH_NODE));
				strcpy((char*)batch_node.barcode,(char const*)barcode);
				ret = record_add((unsigned char*)&batch_node);
				if (ret)
				{
					//��¼����ʧ�ܣ�������ʾ���û�
					//@todo...
					hw_platform_beep_ctrl(400,4000);
					OSTimeDlyHMSM(0,0,0,50);
					hw_platform_beep_ctrl(400,4000);
				}
				break;
			case EVENT_SCAN_KEY_LONG_PRESS:
				//hw_platform_stop_led_blink(LED_GREEN);
				//�л���memory���������ģʽ
				device_current_state = STATE_Memory_Mode_BT_WaitPair;
				enter_into_BT_Mode(STATE_BT_Mode_WaitPair,0);
				break;
			case EVENT_BT_CONNECTED:
				//�л���memory����������ģʽ
				if (g_param.device_state == DEVICE_MODE_SWITCH)
				{
					exit_from_Memory_Mode();
					device_current_state = STATE_BT_Mode_Connect;
				}
				else
				{
					device_current_state = STATE_Memory_Mode_BT_Connect;
				}
				
				enter_into_BT_Mode(STATE_BT_Mode_Connect,0);
				break;
			case EVENT_BT_DISCONNECTED:
				break;
			//case EVENT_USB_CABLE_INSERT:
			//	//�л���USB HIDģʽ
			//	exit_from_Memory_Mode();
			//	last_state = STATE_Memory_Mode;
			//	device_current_state = STATE_HID_Mode;
			//	enter_into_USB_HID_Mode();
			//	break;
			//case EVENT_USB_CABLE_REMOVE:
			//	break;
			case EVENT_LOW_POWER:
				lowpower_tip();
				break;
			default:
				break;
			}
		}
		else if(device_current_state ==  STATE_Memory_Mode_BT_Connect)
		{
			switch(event)
			{
				//case EVENT_SCAN_KEY_SINGLE_CLICK:
				//ɨ������
				//	ret = scanner_get_barcode(barcode,MAX_BARCODE_LEN,codetype,&codelen);
				//	hw_platform_stop_led_blink(LED_GREEN);
				//	if (ret != 0)
				//	{
				//		break;
				//	}
			case EVENT_SCAN_GOT_BARCODE:
				scan_barcode_ok_tip();
				if (lowpower_state)
				{
					lowpower_tip();
				}
				if (barcode_pre_handle(barcode))
				{
					break;
				}
				//ɨ�赽������
				//����ȡ�������뱣�浽memory
				memset((void*)&batch_node,0,sizeof(TBATCH_NODE));
				strcpy((char*)batch_node.barcode,(char const*)barcode);
				ret = record_add((unsigned char*)&batch_node);
				if (ret)
				{
					//��¼����ʧ�ܣ�������ʾ���û�
					//@todo...
					hw_platform_beep_ctrl(400,4000);
					OSTimeDlyHMSM(0,0,0,50);
					hw_platform_beep_ctrl(400,4000);
				}
				break;
			case EVENT_SCAN_KEY_LONG_PRESS:
				//hw_platform_stop_led_blink(LED_GREEN);
				//�л���memory���������ģʽ
				device_current_state = STATE_Memory_Mode_BT_WaitPair;
				enter_into_BT_Mode(STATE_BT_Mode_WaitPair,0);
				break;
			case EVENT_BT_CONNECTED:
				break;
			case EVENT_BT_DISCONNECTED:
				//�л���memory������δ����ģʽ
				device_current_state = STATE_Memory_Mode;
				enter_into_BT_Mode(STATE_BT_Mode_Disconnect,0);
				break;
				//case EVENT_USB_CABLE_INSERT:
				//	//�л���USB HIDģʽ
				//	exit_from_Memory_Mode();
				//	last_state = STATE_Memory_Mode;
				//	device_current_state = STATE_HID_Mode;
				//	enter_into_USB_HID_Mode();
				//	break;
				//case EVENT_USB_CABLE_REMOVE:
				//	break;
			case EVENT_LOW_POWER:
				lowpower_tip();
				break;
			default:
				break;
			}
		}
		else if(device_current_state ==  STATE_Memory_Mode_BT_WaitPair)
		{
			switch(event)
			{
			case EVENT_SCAN_KEY_SINGLE_CLICK:
			case EVENT_SCAN_KEY_LONG_PRESS:
				exit_from_BT_Mode(STATE_BT_Mode_WaitPair);
				device_current_state = STATE_Memory_Mode;
				enter_into_BT_Mode(STATE_BT_Mode_Disconnect,0);
				break;
			case EVENT_SCAN_GOT_BARCODE:
				scan_barcode_ok_tip();
				if (lowpower_state)
				{
					lowpower_tip();
				}
				if (barcode_pre_handle(barcode))
				{
					break;
				}
				//ɨ�赽������
				//����ȡ�������뱣�浽memory
				memset((void*)&batch_node,0,sizeof(TBATCH_NODE));
				strcpy((char*)batch_node.barcode,(char const*)barcode);
				ret = record_add((unsigned char*)&batch_node);
				if (ret)
				{
					//��¼����ʧ�ܣ�������ʾ���û�
					//@todo...
					hw_platform_beep_ctrl(400,4000);
					OSTimeDlyHMSM(0,0,0,50);
					hw_platform_beep_ctrl(400,4000);
				}
				break;
			case EVENT_BT_CONNECTED:
				//�л���memory����������ģʽ
				exit_from_BT_Mode(STATE_BT_Mode_WaitPair);
				device_current_state = STATE_Memory_Mode_BT_Connect;
				enter_into_BT_Mode(STATE_BT_Mode_Connect,0);
				break;
			case EVENT_BT_DISCONNECTED:
				break;
				//case EVENT_USB_CABLE_INSERT:
				//	//�л���USB HIDģʽ
				//	exit_from_Memory_Mode();
				//	last_state = STATE_Memory_Mode;
				//	device_current_state = STATE_HID_Mode;
				//	enter_into_USB_HID_Mode();
				//	break;
				//case EVENT_USB_CABLE_REMOVE:
				//	break;
			case EVENT_LOW_POWER:
				lowpower_tip();
				break;
			default:
				break;
			}
		}
	}
}

/**
* @brief	��ȡ�����첽�¼����߳�
* @note     ����һЩ�첽�¼��ļ�����жϷ��������post����������߳�post����
*			���磺�����¼����ڶ�ʱ���жϵķ������post����
*				  USB�ߵĲ��루ʵ������USB HID device��ö�ٳɹ�������USB���жϷ���������
*				  �������ӵ�״̬�仯��������ģ���ά���̸߳���
*				  ���߳�ֻ����һЩ�����Ե�״̬����¼�����ص����͡�USB���Ƿ񱻰γ�
*/
void Event_capture_thread(void *p)
{
#ifdef DEBUG_VER
	printf("Enter into Event_capture_thread!\r\n");
#endif
	while (1)
	{
		//{
			if(hw_platform_USBcable_Insert_Detect())
			{
				//���ڳ��
				if (hw_platform_ChargeState_Detect())
				{
					//������
					hw_platform_led_ctrl(LED_RED,0);
				}
				else
				{
					hw_platform_led_ctrl(LED_RED,1);
				}
			}
			else
			{
				hw_platform_led_ctrl(LED_RED,0);
				//�жϵ�ص�����
				if (hw_platform_get_PowerClass() == 0)
				{
					lowpower_cnt++;
					if (lowpower_cnt>10)
					{
#ifdef DEBUG_VER
						printf("low power detected!\r\n");
#endif
						if (lowpower_state == 0)
						{
							OSQPost(pEvent_Queue,(void*)EVENT_LOW_POWER);
							lowpower_state = 1;
						}

					}
				}
				else
				{
					lowpower_cnt = 0;
				}
			}
			

//			if (bDeviceState == CONFIGURED)
//			{
//#ifdef DEBUG_VER
//				printf("USB HID Enum OK detected!\r\n");
//#endif
//				OSQPost(pEvent_Queue,(void*)EVENT_USB_CABLE_INSERT);
//			}
			OSTimeDlyHMSM(0,0,0,50);	//50ms��Ƶ�����д��߳�
		//}
	}
}


/*
 * @brief ����ģ��ά���߳�
 * @note  ���߳���Ҫ�����������
 *        1 : �������ģ���Ƿ��з�������״̬�仯��ָʾ�źŻ����������⵽�ˣ������¼���Ϣ����
 *        2 ������Ƿ���������Ҫͨ������ģ�鷢�ͣ�����оͷ��ͳ�ȥ
*/
void BT_Daemon_thread(void *p)
{
#if(BT_MODULE == USE_BT816)
	static unsigned int last_status;
#endif
	int ret;
    unsigned int len;
	unsigned char	err;
	unsigned char	*pbarcode;

#if(BT_MODULE == USE_WBTDS01)
	ret = WBTD_init();
#else
	ret = BT816_init();
	last_status = BT_MODULE_STATUS_DISCONNECT;
#endif
	if (ret)
	{
#if(BT_MODULE == USE_WBTDS01)
		WBTD_Reset();
		ret = WBTD_init();
#else
		ret = BT816_init();
#endif
		if (ret)
		{
			hw_platform_beep_ctrl(400,4000);
			OSTimeDlyHMSM(0,0,0,50);
			hw_platform_beep_ctrl(400,4000);
		}
		assert(ret == 0);
	}

#ifdef DEBUG_VER
	printf("BT Module init Success!\r\n");
#endif

#if(BT_MODULE == USE_BT816)
	//BT816_connect_last_host();		//��ͼ�������һ�ε���������
#endif


	//for test SPP mode
	//while(1)
	//{
	//	if (spp_buffer_head)
	//	{
	//		printf("spp reclen=%d\r\n",spp_buffer_head);

	//		for (len = 0; len < spp_buffer_head;len++)
	//		{
	//			printf("0x%x,",spp_rec_buffer[len]);
	//		}
	//		printf("\r\n");

	//		spp_buffer_head = 0;
	//	}

	//	OSTimeDlyHMSM(0,0,0,50);
	//}

	while (1)
	{
#if(BT_MODULE == USE_WBTDS01)
		ret = WBTD_got_notify_type();
		if ((ret == BT_MODULE_STATUS_CONNECTED) || (ret == BT_MODULE_STATUS_DISCONNECT))
		{
#ifdef DEBUG_VER
			printf("BT Module Status = %s!\r\n",(ret==1)?"Connected":"Disconnect");
#endif
			OSQPost(pEvent_Queue,(void*)((ret == BT_MODULE_STATUS_CONNECTED)?EVENT_BT_CONNECTED:EVENT_BT_DISCONNECTED));
		}
#else
		ret = BT816_connect_status();
		if ((ret == BT_MODULE_STATUS_CONNECTED)||(ret == BT_MODULE_STATUS_DISCONNECT))
		{
			if (ret != last_status)
			{
				last_status = ret;

#ifdef DEBUG_VER
				printf("BT Module Status = %s!\r\n",(ret==BT_MODULE_STATUS_CONNECTED)?"Connected":"Disconnect");
#endif
				OSQPost(pEvent_Queue,(void*)((ret == BT_MODULE_STATUS_CONNECTED)?EVENT_BT_CONNECTED:EVENT_BT_DISCONNECTED));
			}
			else
			{
				if ((ret == BT_MODULE_STATUS_DISCONNECT)&&(device_current_state == STATE_BT_Mode_Disconnect || device_current_state == STATE_Memory_Mode))
				{
					//����һ����ֵ����ͼ������������
					BT816_connect_last_host();
				}
			}
		}
#endif


		pbarcode = (unsigned char*)OSQPend(pBarcode_Queue,20,&err);
		if (pbarcode)
		{
#ifdef DEBUG_VER
			printf("BT Module got data(%s) to send!\r\n",pbarcode);
#endif
#if(BT_MODULE == USE_WBTDS01)
			if (WBTD_hid_send(pbarcode,strlen((char const*)pbarcode),&len))
#else
			if (BT816_send(pbarcode,strlen((char const*)pbarcode)))
#endif
			{
				//����ʧ��Ӧ����ô����ʲô��������ô????!!!!
				//@todo...
#ifdef DEBUG_VER
				printf("BT Module send data Fail!\r\n");
#endif
			}
			else
			{
				pull_barcode_from_cash(pbarcode);
				//OSTimeDlyHMSM(0,0,0,50);

#ifdef DEBUG_VER
				printf("BT Module send data Success!\r\n");
#endif
			}
		}
	}
}


/*
 * @brief����U��ģʽ
*/
//void u_disk_proc(void)
//{
//	//g_mass_storage_device_type = MASSTORAGE_DEVICE_TYPE_SPI_FLASH;
//	//usb_device_init(USB_MASSSTORAGE);
//
//	OSSchedLock();
//
//	while(hw_platform_USBcable_Insert_Detect() == 1)
//	{	
//		if(bDeviceState != CONFIGURED)
//		{
//			break;
//		}
//
//		delay_ms(1);
//	}
//
//	OSSchedUnlock();
//}

/*
 * @brief �͵�����ʾ,��Ƴ���5S����˸
 * @return 0:��ʾ�ڼ�û��USB�ߵĲ���		1:��ʾ�ڼ���USB�ߵĲ���
*/
int lowpower_tip(void)
{
	int i;
	OSSchedLock();
	hw_platform_start_led_blink(LED_RED,5);
	for (i = 0; i<15;i++)
	{
		if ((i%5) == 0)
		{
			hw_platform_beep_ctrl(400,4000);
		}
		if (hw_platform_USBcable_Insert_Detect())
		{
			hw_platform_stop_led_blink(LED_RED);
			OSSchedUnlock();
			return 1;
		}
	}
	hw_platform_stop_led_blink(LED_RED);
	OSSchedUnlock();
	return 0;
}

/*
 * @brief ϵͳ�������ʾ
*/
void system_err_tip(void)
{
	while(1)
	{
		//@todo...
	}
}


// Cortex System Control register address
#define SCB_SysCtrl					((u32)0xE000ED10)
// SLEEPDEEP bit mask
#define SysCtrl_SLEEPDEEP_Set		((u32)0x00000004)

/*
 * @brief ��ʼ���߳�
 */
void app_init_thread(void *p)
{
	int ret;
#ifdef DEBUG_VER
	printf("app init thread startup...\r\n");
#endif

	OS_CPU_SysTickInit();

	app_init();

	Keypad_Init();

	ret = record_module_init();
	if (ret != 0)
	{
		system_err_tip();
	}

	if (recover_record_by_logfile())
	{
		system_err_tip();
	}

	if (ReadTerminalPara())
	{
		if (DefaultTerminalPara())
		{
			system_err_tip();
		}
	}

	if (g_param.device_state == DEVICE_MODE_MEM || g_param.device_state == DEVICE_MODE_SWITCH)
	{
		device_current_state = STATE_Memory_Mode;	//�ѻ�״̬
		hw_platform_led_ctrl(LED_GREEN,1);
	}
	else
	{
		device_current_state = STATE_BT_Mode_Disconnect;	//����ģʽδ����״̬
		hw_platform_start_led_blink(LED_BLUE,150);
	}

	scanner_mod_init();

	//usb_device_init(USB_KEYBOARD);

	OSTaskCreateExt(State_Machine_thread,
		(void *)0,
		&thread_statemachine_stk[STACK_SIZE_TASKSM-1],
		8,
		8,
		&thread_statemachine_stk[0],
		STACK_SIZE_TASKSM,
		(void *)0,
		(INT16U)(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));

	OSTaskCreateExt(Event_capture_thread,
		(void *)0,
		&thread_eventcapture_stk[STACK_SIZE_TASKEC-1],
		7,
		7,
		&thread_eventcapture_stk[0],
		STACK_SIZE_TASKEC,
		(void *)0,
		(INT16U)(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));

	OSTaskCreateExt(BT_Daemon_thread,
		(void *)0,
		&thread_bt_stk[STACK_SIZE_TASKBT-1],
		6,
		6,
		&thread_bt_stk[0],
		STACK_SIZE_TASKBT,
		(void *)0,
		(INT16U)(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));

	OSTimeDlyHMSM(0,0,0,10);
	OSTaskDel(OS_PRIO_SELF);
}

/*
 * @brief ����Ӧ��
 */
void app_startup(void)
{
	memset((void*)thread_eventcapture_stk, 0xAA, sizeof(thread_eventcapture_stk));
	memset((void*)thread_statemachine_stk, 0xBB, sizeof(thread_statemachine_stk));
	memset((void*)thread_bt_stk, 0xCC, sizeof(thread_bt_stk));

	OSInit();

	OSDebugInit();

	p_init_thread_stk = (OS_STK*)Jmalloc(STACK_SIZE_TASKINI*sizeof(OS_STK));
	assert(p_init_thread_stk != 0);

	OSTaskCreateExt(app_init_thread,
		(void *)0,
		&p_init_thread_stk[STACK_SIZE_TASKINI-1],
		5,
		5,
		&p_init_thread_stk[0],
		STACK_SIZE_TASKINI,
		(void *)0,
		(INT16U)(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));

	OSStart();
}

