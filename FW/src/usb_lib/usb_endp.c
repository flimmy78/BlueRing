/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V2.2.1
* Date               : 09/22/2008
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "PCUsart.h"
#include "usb_bot.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//u8 buffer_out[VIRTUAL_COM_PORT_DATA_SIZE];
//u32 count_out = 0;
u32 count_in = 0;

extern unsigned char g_usb_type; 

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : EP3_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
 // int i;

  //if (g_usb_type == USB_VIRTUAL_PORT)
  //{
	 // count_out = GetEPRxCount(ENDP3);
	 // PMAToUserBufferCopy(buffer_out, ENDP3_RXADDR, count_out);
	 // SetEPRxValid(ENDP3);

	 // for (i = 0; i < count_out; i++)
	 // {
		//  PCUsart_InByte(buffer_out[i]);
	 // }
  //}
}

/*******************************************************************************
* Function Name  : EP2_OUT_Callback.
* Description    : EP2 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP2_OUT_Callback(void)
{
	if (g_usb_type == USB_MASSSTORAGE)
	{
		Mass_Storage_Out();
	}
}

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
	if (g_usb_type == USB_MASSSTORAGE)
	{
		Mass_Storage_In();
	}
	else
	{
		count_in = 0;
	}
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

