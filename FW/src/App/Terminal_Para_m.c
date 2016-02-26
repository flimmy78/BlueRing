/**
 * @file  Terminal_Para.c
 * @brief T1Gen��Ŀ���ն˲�������ģ��
 * @version 1.0
 * @author joe
 * @date 2011��03��30��
 * @note
*/  
/* Private include -----------------------------------------------------------*/
#include "Terminal_Para.h"
#include <string.h>
#include "crc32.h"
#include "record_mod.h"



/* private Variable -----------------------------------------------*/
/* external Variable -----------------------------------------------*/

/* Global Variable -----------------------------------------------*/
//�����ն˵�ϵͳ����
TTerminalPara		g_param;				//�ն˲���

static unsigned char param_mod_state = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private function  -----------------------------------------------*/

/**
* @brief	�ӱ����ն�ϵͳ�����Ĵ洢ģ���ж�ȡ�������浽ȫ�ֱ�����
* @param[in]  none				
* @param[out] �洢�ն˲�����ȫ�ֱ���
* @return     unsigned char  0  :SUCCESS   else : �������
* @note                  
*/
int ReadTerminalPara(void)
{
	unsigned long			checkvalue;
	int		ret;

	if (param_mod_state == 0)
	{
		ret = param_init(sizeof(TTerminalPara));
		if (ret)
		{
			if (ret == -3 || ret == -4 || ret == -6)
			{
				ret = param_format(sizeof(TTerminalPara));
				if (ret)
				{
					return ret;
				}
			}
			else
			{
				return ret;
			}
		}

		param_mod_state = 1;
	}

	ret = param_read((unsigned char*)&g_param,sizeof(TTerminalPara));
	if(ret)
	{
		return ret;
	}



#if 1
	//����У��ֵ�Ƿ���ȷ
	checkvalue = crc32(0,&g_param.struct_ver,sizeof(TTerminalPara) - 4);
	if (g_param.checkvalue != checkvalue)
	{
		//������У��ֵ����
		return 2;
	}

	// �������Ƿ���ȷ
	if ((g_param.endtag[0] != 0x55)||(g_param.endtag[1] != 0xAA)||(g_param.struct_ver != 1))
	{
		//�����Ľ�����ǲ���
		return 3;
	}

	if ((g_param.bt_mode != BT_MODE_HID)&&(g_param.bt_mode != BT_MODE_SPP))
	{
		return 4;
	}

	if ((g_param.kb_language != KB_LAN_US)&&(g_param.kb_language != KB_LAN_FR)&&(g_param.kb_language != KB_LAN_IT)&&(g_param.kb_language != KB_LAN_GE))
	{
		return 5;
	}

	if ((g_param.device_state != DEVICE_MODE_BT)&&(g_param.device_state != DEVICE_MODE_MEM)&&(g_param.device_state != DEVICE_MODE_SWITCH))
	{
		return 6;
	}

	//�����������Ƿ���ȷ
	//@todo....
#endif
	return 0;	
}

/**
* @brief	���û��������ն�ϵͳ�������浽�洢ģ��
* @param[in]   none				
* @param[in] �洢�ն˲�����ȫ�ֱ���
* @return     none
* @note   �˺�����ʵ�ֵ��ǽ��������浽PSAM����                
*/
int SaveTerminalPara(void)
{
	int					ret;

	// ���¼���У��        
	g_param.checkvalue = crc32(0,&g_param.struct_ver, sizeof(TTerminalPara)-4);

	ret = param_write((unsigned char*)&g_param,sizeof(TTerminalPara));
	return ret;
}


/**
* @brief ����Ĭ�ϵĲ��Բ���������
*/
int DefaultTerminalPara(void)
{
	unsigned char		*pSrc;
	//unsigned char		i,tmp[2];

	pSrc				= (unsigned char *)&g_param;
	//��������в���
	memset(pSrc,0,sizeof(TTerminalPara));
	//���ý������
	g_param.endtag[0]		= 0x55;
	g_param.endtag[1]		= 0xAA;
	//���ò����ṹ�汾��
	g_param.struct_ver		= 0x01;

	
	//����Ĭ�ϵ��ն˲���
	g_param.kb_language = KB_LAN_US;			//Ĭ�ϵļ���������Ӣ��		
	//g_param.power_save_mode = 1;				//Ĭ��֧�ֽ���ģʽ
	g_param.lower_power_timeout = TIMEOUT_60S;	//Ĭ��1���ӽ���͹���ģʽ
	g_param.ios_softkeypad_enable = 1;			//Ĭ��֧��IOSС���̵ĵ���
	g_param.bt_mode = BT_MODE_HID;				//BTģ��Ĭ�Ϲ�����HIDģʽ
	g_param.device_state = DEVICE_MODE_BT;		//Ĭ�Ϲ���������ģʽ
	g_param.auto_con_last_host = 1;				

	return SaveTerminalPara();
}

