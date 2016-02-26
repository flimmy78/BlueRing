/**
* @file WBTDS01.c
* @brief WBTDS01 蓝牙模块的驱动
* @version V0.0.1
* @author joe.zhou
* @date 2015年08月31日
* @note
* @copy
*
* 此代码为深圳合杰电子有限公司项目代码，任何人及公司未经许可不得复制传播，或用于
* 本公司以外的项目。本司保留一切追究权利。
*
* <h1><center>&copy; COPYRIGHT 2015 heroje</center></h1>
*
*/
#include "WBTDS01.h"
#include "ucos_ii.h"
#include "stm32f10x_lib.h"
#include "string.h"
#include <assert.h>

//#define	WBTD_DEBUG

#define WBTD_RESPONSE_INIT				0

//通知类型的响应数据状态
#define WBTD_RESPONSE_NOTIFY_BEGIN		1
#define WBTD_RESPONSE_NOTIFY_COMPLETE	2

//应答类型的响应数据状态
#define WBTD_RESPONSE_ANS_BEGIN			3
#define WBTD_RESPONSE_ANS_COMPLETE		4

#define WBTD_RESPONSE_FULL				5

/**
* @brief WBTDS01响应定义  WBTDS01->host
*/
typedef struct {
	unsigned short			DataPos;
	unsigned short			DataLength;
	unsigned char			status;
	unsigned char			*DataBuffer;
}TWBTDRes;

TWBTDRes		wbtd_res;

#define		WBTD_RES_BUFFER_LEN			64		//从WBTDS01的数据手册上看，没有超过64字节的响应数据		
#define     WBTD_NOTIFY_BUFFER_LEN		32		//

static unsigned char	wbtd_recbuffer[WBTD_RES_BUFFER_LEN];
static unsigned char	wbtd_notify_buffer[WBTD_NOTIFY_BUFFER_LEN];	
static unsigned int		got_notify_flag;
//static unsigned int		cmd_send_flag;
/*
 * @brief: 初始化模块端口
 * @note 使用串口2
*/
static void WBTD_GPIO_config(unsigned int baudrate)
{
	GPIO_InitTypeDef				GPIO_InitStructure;
	USART_InitTypeDef				USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	//B-Reset  PG.11
	GPIO_InitStructure.GPIO_Pin				= GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed			= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode			= GPIO_Mode_Out_PP;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_SetBits(GPIOG, GPIO_Pin_11);

	// 使用UART3, PB10,PB11
	/* Configure USART3 Tx (PB.10) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin				= GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed			= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode			= GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure USART3 Rx (PB.11) as input floating				*/
	GPIO_InitStructure.GPIO_Pin				= GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode			= GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate		= baudrate;					
	USART_InitStructure.USART_WordLength	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits		= USART_StopBits_1;
	USART_InitStructure.USART_Parity		= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode			= USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
}

/*
 * @brief: 串口中断的初始化
*/
static void WBTD_NVIC_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Enable the USART3 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel				=USART3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority	= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd			= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ClearITPendingBit(USART3, USART_IT_RXNE); 
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}


/**
* @brief  发数据给蓝牙模块
* @param[in] unsigned char *pData 要发送的数据
* @param[in] int length 要发送数据的长度
*/
static void send_data_to_WBTDS01(const unsigned char *pData, unsigned short length)
{
	while(length--)
	{
		USART_SendData(USART3, *pData++);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
		{
		}
	}
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET){};
}


/*
 * @brief 清空接收蓝牙模块响应数据的buffer
*/
static void WBTD_reset_resVar(void)
{
	wbtd_res.DataPos = 0;
	wbtd_res.DataLength = 0;
	wbtd_res.status	 = WBTD_RESPONSE_INIT;
}


/**
* @brief 处理host收到WBTD的数据
* @param[in] unsigned char c 读入的字符
* @return 0:success put in buffer
*        -1:fail
*/
int WBTD_RxISRHandler(unsigned char c)
{
#ifdef WBTD_DEBUG
	wbtd_res.DataBuffer[wbtd_res.DataPos++] = c;

	if (wbtd_res.DataPos == 20)
	{
		wbtd_res.DataPos = 0;
		wbtd_res.status	= WBTD_RESPONSE_ANS_COMPLETE;
	}
#else
	if (wbtd_res.DataPos == WBTD_RES_BUFFER_LEN)
	{
		wbtd_res.status = WBTD_RESPONSE_FULL;
	}

	if(wbtd_res.status == WBTD_RESPONSE_FULL)
	{
		return -1;
	}

	if (wbtd_res.status == WBTD_RESPONSE_INIT || wbtd_res.status == WBTD_RESPONSE_NOTIFY_COMPLETE \
		|| wbtd_res.status == WBTD_RESPONSE_ANS_COMPLETE)
	{
		wbtd_res.DataPos = 0;
		wbtd_res.DataBuffer[wbtd_res.DataPos++] = c;
		if(c == '+')
		{
			got_notify_flag = 0;
			wbtd_res.status = WBTD_RESPONSE_NOTIFY_BEGIN;
		}
		else if ((c == 0x0d)&&(c==0x0a))
		{
			wbtd_res.DataPos--;		//第一个字节就是0x0d或者0x0a，丢弃
		}
		else
		{
			wbtd_res.status = WBTD_RESPONSE_ANS_BEGIN;
		}
	}
	else
	{
		wbtd_res.DataBuffer[wbtd_res.DataPos++] = c;
		if (c == 0x0d || c == 0x0a)
		{
			if (wbtd_res.status == WBTD_RESPONSE_NOTIFY_BEGIN)
			{
				wbtd_res.status = WBTD_RESPONSE_NOTIFY_COMPLETE;
				got_notify_flag = 1;
				memcpy(wbtd_notify_buffer,wbtd_res.DataBuffer,wbtd_res.DataPos-1);
				wbtd_notify_buffer[wbtd_res.DataPos-1] = 0;
			}
			else
			{
				wbtd_res.status = WBTD_RESPONSE_ANS_COMPLETE;
			}
		}
	}
#endif
	return 0;
}


/**
* @brief  发命令给蓝牙模块WBTDS01,并等待响应结果
* @param[in] unsigned char *pData 要发送的数据
* @param[in] unsigned int	length 要发送数据的长度
* @return		0: 成功
*				-1: 失败
*/
static int WBTD_write_cmd(const unsigned char *pData, unsigned int length)
{
	unsigned int	wait_cnt;
	send_data_to_WBTDS01(pData, length);
	WBTD_reset_resVar();
	//cmd_send_flag = 1;
	wait_cnt = 50;
	while (wait_cnt)
	{
		if (wbtd_res.status == WBTD_RESPONSE_ANS_COMPLETE)
		{
			//cmd_send_flag = 0;
			return 0;
		}
		else if (wbtd_res.status == WBTD_RESPONSE_FULL)
		{
			//cmd_send_flag = 0;
			return -1;
		}

		OSTimeDlyHMSM(0,0,0,20);
		wait_cnt--;
	}

	return -1;
}


//const unsigned char	*query_version_cmd="AT+VER=?";
//const unsigned char	*set_device_name_cmd="AT+DNAME=%s";

/*
 * @brief 蓝牙模块WBTDS01的复位
*/
void WBTD_Reset(void)
{
	//拉低复位信号100ms
	GPIO_ResetBits(GPIOG, GPIO_Pin_11);
	OSTimeDlyHMSM(0,0,0,100);
	GPIO_SetBits(GPIOG, GPIO_Pin_11);

	WBTD_reset_resVar();

	got_notify_flag = 0;
	memset(wbtd_notify_buffer,0,WBTD_NOTIFY_BUFFER_LEN);
	OSTimeDlyHMSM(0,0,0,100);
}

/*
 * @brief 查询蓝牙模块WBTDS01的版本号
 * @param[out]  unsigned char *ver_buffer  返回查询到的版本号，如果为空表示查询失败
*/
int WBTD_query_version(unsigned char *ver_buffer)
{
	unsigned char	buffer[21];
	int		ret;

	assert(ver_buffer != 0);
	ver_buffer[0] = 0;
	memcpy(buffer,"AT+VER=?\x0d\x0a",10);
	ret = WBTD_write_cmd((const unsigned char*)buffer,10);
	if (ret)
	{
		return -1;
	}

	if ((wbtd_res.DataLength > 1)&&((wbtd_res.DataBuffer[0] == 'V')||(wbtd_res.DataBuffer[0] == 'v')))
	{
		if (wbtd_res.DataLength < 22)
		{
			memcpy(ver_buffer,wbtd_res.DataBuffer,wbtd_res.DataLength-1);
			ver_buffer[wbtd_res.DataLength-1] = 0;
		}
		else
		{
			memcpy(ver_buffer,wbtd_res.DataBuffer,20);
			ver_buffer[20] = 0;
		}

		return 0;
	}

	return -1;
}


/*
 * @brief 设置蓝牙模块的设备名称
 * @param[in]  unsigned char *name  设置的名称,字符串
 * @return 0: 设置成功		else：设置失败
 * @note 从手册暂时没有看到支持的名称的最大长度是多少，所以如果设置的名称太长可能会设置失败
 *       在此接口中将设备名称限定为最长支持20个字节
*/
int WBTD_set_name(unsigned char *name)
{
	unsigned char	buffer[31];
	int		ret,len;

	assert(name != 0);
	memcpy(buffer,"AT+DNAME=",9);
	len = strlen((char const*)name);
	if (len>20)
	{
		memcpy(buffer+9,name,20);
		buffer[29] = 0x0d;
		buffer[30] = 0x0a;
		len = 31;
	}
	else
	{
		memcpy(buffer+9,name,len);
		buffer[9+len] = 0x0d;
		buffer[10+len] = 0x0a;
		len = 11+len;
	}
	ret = WBTD_write_cmd((const unsigned char*)buffer,len);
	if (ret)
	{
		return -1;
	}

	if ((wbtd_res.DataLength == 3)&&(memcmp(wbtd_res.DataBuffer,"OK\x0d",3) == 0))
	{
		return 0;
	}

	return -1;
}

/*
 * @brief 设置蓝牙模块的设备名称
 * @param[in]  unsigned char *name  设置的名称,字符串
 * @return 0: 设置成功		else：设置失败
 * @note 从手册暂时没有看到支持的名称的最大长度是多少，所以如果设置的名称太长可能会设置失败
 *       在此接口中将设备名称限定为最长支持20个字节
*/
int WBTD_set_baudrate(WBTD_BAUDRATE baudrate)
{
	unsigned char	buffer[20];
	int		ret,len;

	memcpy(buffer,"AT+URATE=",9);
	len = 16;
	switch(baudrate)
	{
	case BAUDRATE_9600:
		memcpy(buffer+9,"9600",4);
		buffer[13] = 0x0d;
		buffer[14] = 0x0a;
		len = 15;
		break;
	case BAUDRATE_19200:
		memcpy(buffer+9,"19200",5);
		buffer[14] = 0x0d;
		buffer[15] = 0x0a;
		break;
	case BAUDRATE_38400:
		memcpy(buffer+9,"38400",5);
		buffer[14] = 0x0d;
		buffer[15] = 0x0a;
		break;
	case BAUDRATE_43000:
		memcpy(buffer+9,"43000",5);
		buffer[14] = 0x0d;
		buffer[15] = 0x0a;
		break;
	case BAUDRATE_57600:
		memcpy(buffer+9,"57600",5);
		buffer[14] = 0x0d;
		buffer[15] = 0x0a;
		break;
	case BAUDRATE_115200:
		memcpy(buffer+9,"115200",6);
		buffer[15] = 0x0d;
		buffer[16] = 0x0a;
		len = 17;
		break;
	}
	ret = WBTD_write_cmd((const unsigned char*)buffer,len);
	if (ret)
	{
		return -1;
	}

	if ((wbtd_res.DataLength == 3)&&(memcmp(wbtd_res.DataBuffer,"OK\x0d",3) == 0))
	{
		return 0;
	}

	return -1;
}

/*
 * @brief 使蓝牙模块进入配对模式
 * @param[in]  unsigned char mode  配对模式  
 *			1: 蓝牙模块会试图重连之前的蓝牙主机   
 *			0：蓝牙模块不会试图重连之前的蓝牙主机   
 * @return 0: 设置成功		else：设置失败
*/
int WBTD_set_autocon(unsigned char mode)
{
	unsigned char	buffer[15];
	int		ret;

	memcpy(buffer,"AT+AUTOCON=",11);
	if (mode)
	{
		buffer[11] = '1';
	}
	else
	{
		buffer[11] = '0';
	}
	buffer[12] = 0x0d;
	buffer[13] = 0x0a;
	ret = WBTD_write_cmd((const unsigned char*)buffer,14);
	if (ret)
	{
		return -1;
	}

	if ((wbtd_res.DataLength == 3)&&(memcmp(wbtd_res.DataBuffer,"OK\x0d",3) == 0))
	{
		return 0;
	}

	return -1;
}

/*
 * @brief 设置蓝牙模块的工作模式(profile = HID、SPP、BLE)
 * @param[in]  unsigned char mode  蓝牙模块的工作模式     
 * @return 0: 设置成功		else：设置失败
*/
int WBTD_set_profile(BT_PROFILE mode)
{
	unsigned char	buffer[20];
	int		ret;

	memcpy(buffer,"AT+PROFILE=",11);
	if (mode == BT_PROFILE_HID)
	{
		buffer[11] = '2';
	}
	else if (mode == BT_PROFILE_SPP)
	{
		buffer[11] = '1';
	}
	else
	{
		buffer[11] = '0';
	}
	buffer[12] = 0x0d;
	buffer[13] = 0x0a;
	ret = WBTD_write_cmd((const unsigned char*)buffer,14);
	if (ret)
	{
		return -1;
	}

	if ((wbtd_res.DataLength == 3)&&(memcmp(wbtd_res.DataBuffer,"OK\x0d",3) == 0))
	{
		return 0;
	}

	return -1;
}

/*
 * @brief 设置蓝牙模块是否使能IOS soft keyboard
 * @param[in]  unsigned char enable  1:Enable   0:Disable
 * @return 0: 设置成功		else：设置失败
*/
int WBTD_set_ioskeypad(unsigned char enable)
{
	unsigned char	buffer[15];
	int		ret;

	memcpy(buffer,"AT+IOSKB=",9);
	if (enable)
	{
		buffer[9] = '1';
	}
	else
	{
		buffer[9] = '0';
	}
	buffer[10] = 0x0d;
	buffer[11] = 0x0a;
	ret = WBTD_write_cmd((const unsigned char*)buffer,12);
	if (ret)
	{
		return -1;
	}

	if ((wbtd_res.DataLength == 3)&&(memcmp(wbtd_res.DataBuffer,"OK\x0d",3) == 0))
	{
		return 0;
	}

	return -1;
}

/*
 * @brief 通过蓝牙模块的HID模式发送ASCII字符串
 * @param[in]  unsigned char *str		需要发送的ASCII字符缓冲
 * @param[in]  unsigned int  len	    待发送字符数
 * @param[out]  unsigned int *send_len  实际发送的字符数
 * @return 0: 发送成功		else：发送失败
*/
int WBTD_hid_send(unsigned char *str,unsigned int len,unsigned int *send_len)
{
	//@todo...
	OSTimeDlyHMSM(0,0,0,20);
        return 0;
}

/*
 * @brief 蓝牙模块WBTDS01的初始化
*/
int WBTD_init(void)
{
	unsigned char	str[21];
	WBTD_GPIO_config(115200);		//default波特率
	WBTD_NVIC_config();

	wbtd_res.DataBuffer = wbtd_recbuffer;
	WBTD_reset_resVar();
	
	got_notify_flag = 0;
	memset(wbtd_notify_buffer,0,WBTD_NOTIFY_BUFFER_LEN);

	if(WBTD_query_version(str))
	{
		return -1;
	}

	if (strcmp((char const*)str,"VER02A")!=0)
	{
		return -2;
	}

	if (WBTD_set_name("H520B Device"))
	{
		return -3;
	}

	if (WBTD_set_profile(BT_PROFILE_HID))
	{
		return -4;
	}

	return 0;
}

/*
 * @brief 蓝牙模块是否获取一个通知消息
 * @return 0:  没有获取到通知消息
 *         1： 获取到连接成功的通知消息
 *         2： 获取到连接断开的通知消息
 *		   3:  unknown message
*/
int WBTD_got_notify_type(void)
{
	if (got_notify_flag)
	{
		got_notify_flag = 0;
		if (strcmp((char const*)wbtd_notify_buffer,"+CON") == 0)
		{
			return 1;
		}
		else if (strcmp((char const*)wbtd_notify_buffer,"+DISCON") == 0)
		{
			return 2;
		}
		else
		{
			return 3;
		}
	}

	return 0;
}

