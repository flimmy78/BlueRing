/**
* @file PCUsart.c
* @brief 手持公交刷卡机与PC交互的串口模块
*
* @version V0.0.1
* @author yinzh
* @date 2010年9月8日
* @note
*   			此模块通信协议参考 非接触式读写器接口规范
* 
* @copy
*
* 此代码为深圳江波龙电子有限公司项目代码，任何人及公司未经许可不得复制传播，或用于
* 本公司以外的项目。本司保留一切追究权利。
*
* <h1><center>&copy; COPYRIGHT 2010 netcom</center></h1>
*/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "PCUsart.h"
#include "ucos_ii.h"
#include "stm32f10x_lib.h"
#include "hw_config.h"
#include "usb_init.h"
#include "TimeBase.h"
#include "Terminal_Para.h"
#include "usb_lib.h"


unsigned char	g_mass_storage_device_type;

extern  unsigned char				g_usb_type;


/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
static void Set_USBClock(void)
{
	/* Enable GPIOA, GPIOD and USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD, ENABLE);

	/* USBCLK = PLLCLK / 1.5 */
	RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
	/* Enable USB clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
static void USB_Interrupts_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN_TX_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void reset_command(void)
{
	//g_pcCommand.CmdPos = 0;
	//g_pcCommand.DataLength = 0;
	//g_pcCommand.status = 0;
	//g_pcCommand.CmdBuffer[0] = 0;
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (unsigned char NewState)
{

}

void PCUsart_Init(void)
{
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();

	//g_pcCommand.CmdBuffer	= g_receive_buff;
	//g_com_handle_flag = 0;

	//reset_command();
}


/**
* @brief 读到一个PC端的响应数据
* @param[in] unsigned char c 读入的字符
* @return 0:success put in buffer
*        -1:fail
*/
int PCUsart_InByte(unsigned char c)
{
#if 0
	int		tmp;
	if (0 == g_pcCommand.status)
	{
		if (g_pcCommand.CmdBuffer[0] == 0x02)
		{
			g_pcCommand.CmdBuffer[g_pcCommand.CmdPos++] = c;
			if (g_pcCommand.CmdPos == 5)
			{
				//接收完长度字节了
				g_pcCommand.DataLength = g_pcCommand.CmdBuffer[1];
				g_pcCommand.DataLength <<= 24;

				tmp = g_pcCommand.CmdBuffer[2];
				tmp <<= 16;
				g_pcCommand.DataLength |= tmp;

				tmp = g_pcCommand.CmdBuffer[3];
				tmp <<= 8;
				g_pcCommand.DataLength |= tmp;

				g_pcCommand.DataLength |= g_pcCommand.CmdBuffer[4];

				if (g_pcCommand.DataLength > TLV_FRAME_SIZE)
				{
					g_pcCommand.CmdBuffer[0] = 0;
				}
			}
			else if (g_pcCommand.CmdPos == g_pcCommand.DataLength+10)
			{
				//数据帧结束了
				g_pcCommand.status = 1;
				return 1;
			}
		}
		else
		{
			g_pcCommand.CmdBuffer[0] = c;
			g_pcCommand.CmdPos = 1;
			g_pcCommand.DataLength = 0;
		}
	}
#endif	
	return 0;

}


/**
* @brief 设备的初始化
* @note
*/
void usb_device_init(unsigned char device_type)
{
	//if ((device_type != USB_VIRTUAL_PORT)&&(device_type != USB_KEYBOARD)&&(device_type != USB_MASSSTORAGE))
	//{
	//	return;
	//}

	if ((device_type != USB_KEYBOARD)&&(device_type != USB_MASSSTORAGE))
	{
		return;
	}

	if (g_usb_type != device_type)
	{
		g_usb_type = device_type;
		PCUsart_Init();
		//vcom_device_state = 0;		//接收到连接命令之前的状态
	}
}