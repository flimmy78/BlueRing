/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : hw_config.c
* Author             : MCD Application Team
* Version            : V2.2.1
* Date               : 09/22/2008
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_prop.h"
#include "TimeBase.h"
#include "PCUsart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//ErrorStatus HSEStartUpStatus;
//EXTI_InitTypeDef EXTI_InitStructure;
unsigned char g_usb_type;

/* Extern variables ----------------------------------------------------------*/
extern u32 count_in;
//extern LINE_CODING linecoding;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Enter_LowPowerMode.
* Description    : Power-off system clocks and power while entering suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode.
* Description    : Restores system clocks and power while exiting suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
}

void SendData_To_PC(unsigned char *pData, int length)
{
	volatile int i = 0;
	//int batch = length/VIRTUAL_COM_PORT_DATA_SIZE;
	//int res	  = length%VIRTUAL_COM_PORT_DATA_SIZE;
	unsigned int	delay_cnt;

	if (g_usb_type == USB_KEYBOARD)	//按键
	{
		count_in = length;
		UserToPMABufferCopy(pData, GetEPTxAddr(ENDP1), count_in);
		SetEPTxCount(ENDP1, count_in);
		/* enable endpoint for transmission */
		SetEPTxValid(ENDP1);
		//StartDelay(600);	//3S的延时
		delay_cnt = 300;
		while (count_in != 0 && delay_cnt != 0 &&(bDeviceState == CONFIGURED))
		{
			delay_cnt--;
			delay_ms(2);
		}
	}
	//else if(g_usb_type == USB_VIRTUAL_PORT)
	//{
	//	for (i = 0; i < batch; i++)
	//	{
	//		count_in	= VIRTUAL_COM_PORT_DATA_SIZE;
	//		UserToPMABufferCopy(pData+i*VIRTUAL_COM_PORT_DATA_SIZE, ENDP1_TXADDR, count_in);
	//		SetEPTxCount(ENDP1, count_in);
	//		SetEPTxValid(ENDP1);
	//		//StartDelay(600);
	//		delay_cnt = 300;
	//		while (count_in != 0 && delay_cnt != 0 && (bDeviceState == CONFIGURED))
	//		{
	//			delay_cnt--;
	//			Delay(20000);
	//		}
	//	}
	//	if (res > 0)
	//	{
	//		//最后一包
	//		count_in	= res;
	//		UserToPMABufferCopy(pData+batch*VIRTUAL_COM_PORT_DATA_SIZE, ENDP1_TXADDR, count_in);
	//		SetEPTxCount(ENDP1, count_in);
	//		SetEPTxValid(ENDP1);
	//		//StartDelay(600);
	//		delay_cnt = 300;
	//		while (count_in != 0 && delay_cnt != 0 &&(bDeviceState == CONFIGURED))
	//		{
	//			delay_cnt--;
	//			Delay(20000);
	//		}
	//	}
	//}//Virtual port
	//else
	//{
	//	if (length > G_SEND_BUF_LENGTH)
	//	{
	//		length = G_SEND_BUF_LENGTH;
	//	}
	//	memcpy(g_send_buff,pData,length);
	//}//USB_Masstorage
	
}

//int usb_test_connect(void)
//{
//	volatile int i = 0;
//	unsigned char pData[2] = {0};
//
//	count_in = 1;
//	UserToPMABufferCopy(pData, GetEPTxAddr(ENDP1), count_in);
//	/* enable endpoint for transmission */
//	SetEPTxValid(ENDP1);
//	while (count_in != 0 && i < 100000)
//	{
//		i++;
//	}
//	if (count_in == 0)
//	{
//		return 0;
//	}
//	return -1;
//}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  u32 Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(u32*)(0x1FFFF7E8);
  Device_Serial1 = *(u32*)(0x1FFFF7EC);
  Device_Serial2 = *(u32*)(0x1FFFF7F0);

  if (Device_Serial0 != 0)
  {
	USB_APP_StringSerial[2] = (u8)(Device_Serial0 & 0x000000FF);
    USB_APP_StringSerial[4] = (u8)((Device_Serial0 & 0x0000FF00) >> 8);
    USB_APP_StringSerial[6] = (u8)((Device_Serial0 & 0x00FF0000) >> 16);
    USB_APP_StringSerial[8] = (u8)((Device_Serial0 & 0xFF000000) >> 24);

    USB_APP_StringSerial[10] = (u8)(Device_Serial1 & 0x000000FF);
    USB_APP_StringSerial[12] = (u8)((Device_Serial1 & 0x0000FF00) >> 8);
    USB_APP_StringSerial[14] = (u8)((Device_Serial1 & 0x00FF0000) >> 16);
    USB_APP_StringSerial[16] = (u8)((Device_Serial1 & 0xFF000000) >> 24);

    USB_APP_StringSerial[18] = (u8)(Device_Serial2 & 0x000000FF);
    USB_APP_StringSerial[20] = (u8)((Device_Serial2 & 0x0000FF00) >> 8);
    USB_APP_StringSerial[22] = (u8)((Device_Serial2 & 0x00FF0000) >> 16);
    USB_APP_StringSerial[24] = (u8)((Device_Serial2 & 0xFF000000) >> 24);
  }
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
