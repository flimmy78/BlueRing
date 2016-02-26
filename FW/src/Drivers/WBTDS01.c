/**
* @file WBTDS01.c
* @brief WBTDS01 蓝牙模块的驱动
* @version V0.0.1
* @author joe.zhou
* @date 2015年08月31日
* @note   根据模块的响应特点，串口驱动采用 DMA发送和接受 + 串口空闲中断判断响应数据接收完的机制。
* @copy
*
* 此代码为深圳合杰电子有限公司项目代码，任何人及公司未经许可不得复制传播，或用于
* 本公司以外的项目。本司保留一切追究权利。
*
* <h1><center>&copy; COPYRIGHT 2015 heroje</center></h1>
*
*/
#include "hw_platform.h"
#if(BT_MODULE == USE_WBTDS01)
#include "WBTDS01.h"
#include "ucos_ii.h"
#include "stm32f10x_lib.h"
#include "string.h"
#include <assert.h>
#include "basic_fun.h"

//#define	WBTD_DEBUG

#define WBTD_RESPONSE_INIT				0

//通知类型的响应数据状态
#define WBTD_RESPONSE_GOT_NOTIFY_CON		0x80
#define WBTD_RESPONSE_GOT_NOTIFY_DISCON		0x40

//应答类型的响应数据状态
#define WBTD_RESPONSE_OK				0x01
#define WBTD_RESPONSE_ERR				0x02

#define WBTD_RESPONSE_UNKOWN			0x04

//响应1: <CR><LF>+ERROR:<Space>3<CR><LF>
//响应2: <CR><LF>+VER:Ver40(jul 14 2015)<CR><LF><CR><LF>OK<CR><LF>
//响应3: 

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

		
#define     WBTD_NOTIFY_BUFFER_LEN		32		//

static unsigned char	wbtd_send_buff[32];

unsigned char	wbtd_recbuffer[WBTD_RES_BUFFER_LEN];

/*
 * @brief: 初始化模块端口
 * @note 使用串口2
*/
/*
 * @brief: 初始化模块端口
 * @note 使用串口2
*/
static void WBTD_GPIO_config(unsigned int baudrate)
{
	GPIO_InitTypeDef				GPIO_InitStructure;
	USART_InitTypeDef				USART_InitStructure;
	DMA_InitTypeDef					DMA_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	//B-Reset  PB.1
	GPIO_InitStructure.GPIO_Pin				= GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed			= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode			= GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_1);

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

	USART_InitStructure.USART_BaudRate		= baudrate;					
	USART_InitStructure.USART_WordLength	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits		= USART_StopBits_1;
	USART_InitStructure.USART_Parity		= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode			= USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);


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

	/* DMA1 Channel7 (triggered by USART2 Tx event) Config */
	DMA_DeInit(DMA1_Channel7);
	DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)(&USART2->DR);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	/* As we will set them before DMA actually enabled, the DMA_MemoryBaseAddr
	 * and DMA_BufferSize are meaningless. So just set them to proper values
	 * which could make DMA_Init happy.
	 */
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)0;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);


	//DMA1通道6配置  
	DMA_DeInit(DMA1_Channel6);  
	//外设地址  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);  
	//内存地址  
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)wbtd_recbuffer;  
	//dma传输方向单向  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  
	//设置DMA在传输时缓冲区的长度  
	DMA_InitStructure.DMA_BufferSize = WBTD_RES_BUFFER_LEN;  
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
	DMA_Init(DMA1_Channel6,&DMA_InitStructure);  

	//使能通道6 
	DMA_Cmd(DMA1_Channel6,ENABLE);  

	//采用DMA方式接收  
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE); 

	/* Enable USART2 DMA Tx request */
	USART_DMACmd(USART2, USART_DMAReq_Tx , ENABLE);

	USART_Cmd(USART2, ENABLE);
}

/*
 * @brief: 串口中断的初始化
*/
static void WBTD_NVIC_config(void)
{
	NVIC_InitTypeDef				NVIC_InitStructure;
	//中断配置  
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);  
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);    

	//配置UART2中断  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;               //通道设置为串口2中断    
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //中断占先等级0    
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;              //中断响应优先级0    
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //打开中断    
	NVIC_Init(&NVIC_InitStructure);  

	/* Enable the DMA1 Channel7 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ClearFlag(DMA1_FLAG_TC7);
}


/**
* @brief  发数据给蓝牙模块
* @param[in] unsigned char *pData 要发送的数据
* @param[in] int length 要发送数据的长度
*/
static void send_data_to_WBTDS01(const unsigned char *pData, unsigned short length)
{
	//while(length--)
	//{
	//	USART_SendData(USART2, *pData++);
	//	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
	//	{
	//	}
	//}
	//while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){};

	/* disable DMA */
	DMA_Cmd(DMA1_Channel7, DISABLE);

	/* set buffer address */
	memcpy(wbtd_send_buff,pData,length);

	DMA1_Channel7->CMAR = (u32)&wbtd_send_buff[0];
	/* set size */
	DMA1_Channel7->CNDTR = length;

	USART_DMACmd(USART2, USART_DMAReq_Tx , ENABLE);
	/* enable DMA */
	DMA_Cmd(DMA1_Channel7, ENABLE);

	while(DMA1_Channel7->CNDTR);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){};
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
int WBTD_RxISRHandler(unsigned char *res, unsigned int res_len)
{
	if (res_len > 4)
	{
		if (memcmp(res,"\x0d\x0a+CON",6) == 0)
		{
			wbtd_res.status &= ~WBTD_RESPONSE_GOT_NOTIFY_DISCON;
			wbtd_res.status |= WBTD_RESPONSE_GOT_NOTIFY_CON;
		}
		else if (memcmp(res,"\x0d\x0a+DISCON",9) == 0)
		{
			wbtd_res.status &= ~WBTD_RESPONSE_GOT_NOTIFY_CON;
			wbtd_res.status |= WBTD_RESPONSE_GOT_NOTIFY_DISCON;
		}
		else if (memcmp(res,"\x0d\x0a+ERROR",8) == 0)
		{
			wbtd_res.DataLength = res_len;
			wbtd_res.status |= WBTD_RESPONSE_ERR; 
		}
		else
		{
			wbtd_res.DataLength = res_len;
			if ((res[res_len-1] == 0x0a)&&(res[res_len-2] == 0x0d)			\
				&&(res[res_len-3] == 'K')&&(res[res_len-4] == 'O'))
			{
				if (wbtd_res.status == WBTD_RESPONSE_INIT)
				{
					wbtd_res.status |= WBTD_RESPONSE_OK;
				}
			}
			else
			{
				wbtd_res.status |= WBTD_RESPONSE_UNKOWN;
			}
		}
	}
}


/**
* @brief  发命令给蓝牙模块WBTDS01,并等待响应结果
* @param[in] unsigned char *pData 要发送的数据
* @param[in] unsigned int	length 要发送数据的长度
* @return		0: 成功
*				-1: 失败
*				-2: 未知响应，可能是响应解析函数有BUG，需要调试
*				-3：响应超时
*/
static int WBTD_write_cmd(const unsigned char *pData, unsigned int length)
{
	unsigned int	wait_cnt;
	send_data_to_WBTDS01(pData, length);
	WBTD_reset_resVar();
	wait_cnt = 200;
	while (wait_cnt)
	{
		if ((wbtd_res.status & WBTD_RESPONSE_OK) == WBTD_RESPONSE_OK)
		{
			return 0;
		}
		else if ((wbtd_res.status & WBTD_RESPONSE_ERR) == WBTD_RESPONSE_ERR)
		{
			return -1;
		}
		else if ((wbtd_res.status & WBTD_RESPONSE_UNKOWN) == WBTD_RESPONSE_UNKOWN)
		{
			return -2;
		}

		OSTimeDlyHMSM(0,0,0,20);
		wait_cnt--;
	}

	return -3;
}


//const unsigned char	*query_version_cmd="AT+VER=?";
//const unsigned char	*set_device_name_cmd="AT+DNAME=%s";

/*
 * @brief 蓝牙模块WBTDS01的复位
*/
void WBTD_Reset(void)
{
	//拉低复位信号100ms
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	OSTimeDlyHMSM(0,0,0,100);
	GPIO_SetBits(GPIOB, GPIO_Pin_1);

	WBTD_reset_resVar();

	//got_notify_flag = 0;
	//memset(wbtd_notify_buffer,0,WBTD_NOTIFY_BUFFER_LEN);
	OSTimeDlyHMSM(0,0,0,100);
}

/*
 * @brief 查询蓝牙模块WBTDS01的版本号
 * @param[out]  unsigned char *ver_buffer  返回查询到的版本号，如果为空表示查询失败
*/
int WBTD_query_version(unsigned char *ver_buffer)
{
	unsigned char	buffer[21];
	int		i,ret;

	assert(ver_buffer != 0);
	ver_buffer[0] = 0;
	memcpy(buffer,"AT+VER?\x0d\x0a",9);
	ret = WBTD_write_cmd((const unsigned char*)buffer,9);
	if (ret)
	{
		return ret;
	}

	if (memcmp(&wbtd_res.DataBuffer[3],"VER:",4) == 0)
	{
		for (i = 0; i < ((wbtd_res.DataLength-15) > 20)?20:(wbtd_res.DataLength-15);i++)
		{
			if (wbtd_res.DataBuffer[7+i] == 0x0d)
			{
				break;
			}

			ver_buffer[i] = wbtd_res.DataBuffer[7+i];
		}
		ver_buffer[i] = 0;
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
	int		len;

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

	return WBTD_write_cmd((const unsigned char*)buffer,len); 
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
	int		len;

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
	return WBTD_write_cmd((const unsigned char*)buffer,len);
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
	return WBTD_write_cmd((const unsigned char*)buffer,14);
}

/*
 * @brief 设置蓝牙模块的工作模式(profile = HID、SPP、BLE)
 * @param[in]  unsigned char mode  蓝牙模块的工作模式     
 * @return 0: 设置成功		else：设置失败
*/
int WBTD_set_profile(BT_PROFILE mode)
{
	unsigned char	buffer[20];

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
	return  WBTD_write_cmd((const unsigned char*)buffer,14);
}

/*
 * @brief 设置蓝牙模块是否使能IOS soft keyboard
 * @param[in]  unsigned char enable  1:Enable   0:Disable
 * @return 0: 设置成功		else：设置失败
*/
int WBTD_set_ioskeypad(unsigned char enable)
{
	unsigned char	buffer[15];

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
	return WBTD_write_cmd((const unsigned char*)buffer,12);
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
	int	ret,i;
	unsigned char	buffer[16];
	unsigned char	tmp[3];

	memcpy(buffer,"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",16);
	*send_len = 0;
	for (i = 0; i < len;i++)
	{
		ascii_to_keyreport(str[i],tmp);
		buffer[0] = tmp[0];
		buffer[2] = tmp[2];
		send_data_to_WBTDS01(buffer, 16);
		*send_len++;

		delay_us(300);
	}

	send_data_to_WBTDS01("\x00\x00\x28\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00", 16);
	delay_us(300);
	return 0;
}


/*
 * @brief 蓝牙模块WBTDS01的初始化
*/
int WBTD_init(void)
{
	unsigned char	str[21];

	wbtd_res.DataBuffer = wbtd_recbuffer;
	WBTD_reset_resVar();
	//got_notify_flag = 0;
	//memset(wbtd_notify_buffer,0,WBTD_NOTIFY_BUFFER_LEN);

	WBTD_GPIO_config(115200);		//default波特率
	WBTD_NVIC_config();

	if(WBTD_query_version(str))
	{
		return -1;
	}

#ifdef DEBUG_VER
	printf("BlueTooth Module Ver:%s\r\n",str);
#endif

	//if (WBTD_set_name("H520B Device"))
	if (WBTD_set_name("WBTDS01"))
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
	if (wbtd_res.status & (WBTD_RESPONSE_GOT_NOTIFY_CON | WBTD_RESPONSE_GOT_NOTIFY_DISCON))
	{
		if (wbtd_res.status & WBTD_RESPONSE_GOT_NOTIFY_CON)
		{
			wbtd_res.status &= 	~WBTD_RESPONSE_GOT_NOTIFY_CON;
			return BT_MODULE_STATUS_CONNECTED;
		}
		else
		{
			wbtd_res.status &= 	~WBTD_RESPONSE_GOT_NOTIFY_DISCON;
			return BT_MODULE_STATUS_DISCONNECT;
		}
	}

	return 0;
}


/*
 * @brief 蓝牙模块HID模式发送测试
*/
int WBTD_hid_send_test(void)
{
	unsigned int i,s;
	for (i = 0; i < 50;i++)
	{
		WBTD_hid_send("12345678901234567890",20,&s);
		delay_ms(150);
	}
}
#endif