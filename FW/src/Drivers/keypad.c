/**
* @file keypad.c
* @brief 按键驱动程序
* @version V0.0.1
* @author joe.zhou
* @date 2015年08月28日
* @note
* @copy
*
* 此代码为深圳合杰电子有限公司项目代码，任何人及公司未经许可不得复制传播，或用于
* 本公司以外的项目。本司保留一切追究权利。
*
* <h1><center>&copy; COPYRIGHT 2015 heroje</center></h1>
*
*/
#include "stm32f10x_lib.h"
#include "ucos_ii.h"
#include <stdio.h>
#include "keypad.h"
#include "cpu.h"
#include "app.h"
#include "TimeBase.h"
#include "hw_platform.h"
#include "HJ5000_scanner.h"

//static  unsigned char		current_press_key;
static  unsigned int		press_cnt;
static  unsigned int		release_cnt; 
static  unsigned char		keypad_state;

//unsigned char	scan_key_trig;


extern	unsigned int	device_current_state;		//设备主状态机
extern	OS_EVENT		*pEvent_Queue;			//事件消息队列
extern unsigned int		keypress_timeout;
/**
* @brief   		Intialize the KeyBoard IO
* @param[in]   none
* @return      none
* @note   
*		按键的硬件连接如下图所示：
*		ScanKey -- PA0
*		//EraseKey -- PB3
*/
static void Keypad_Initport(void)
{
	GPIO_InitTypeDef  gpio_init;
	EXTI_InitTypeDef EXTI_InitStructure;    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//ScanKey  PA.0
	gpio_init.GPIO_Pin   = GPIO_Pin_0;
	gpio_init.GPIO_Speed = GPIO_Speed_10MHz;
	gpio_init.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &gpio_init);		

	/* Connect EXTI Line0 to PA0 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	/* Configure EXTI LineX to generate an interrupt on falling edge */
	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_GenerateSWInterrupt(EXTI_Line0);
	//scan_key_trig = 0;
}

/**
* @brief   	Enable the keypad interrupt
* @return      none
*/
void Keypad_Int_Enable(void)
{
	NVIC_InitTypeDef	NVIC_InitStructure;
	EXTI_ClearITPendingBit(EXTI_Line0);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);    // 抢占式优先级别

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
* @brief   	Disable the keypad interrupt
* @return      none
*/
void Keypad_Int_Disable(void)
{
	NVIC_InitTypeDef	NVIC_InitStructure;
	EXTI_ClearITPendingBit(EXTI_Line0);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/**
* @brief 初始化按键检测需要使用到的定时器,20ms定时器，用以检测按键的按下时长
*/
void Keypad_Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef						TIM_TimeBaseStructure;
	TIM_OCInitTypeDef							TIM_OCInitStructure;
	NVIC_InitTypeDef							NVIC_InitStructure;

	//初始化结构体变量
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);

	/*开启相应时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  


	//定时计算公式：(（1+Prescaler）/72M ) *(Period+1) = ((1+39)/72M) * (72000/2-1+1) = 20ms
	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler			= 39;      
	TIM_TimeBaseStructure.TIM_CounterMode		= TIM_CounterMode_Up; //向上计数
	TIM_TimeBaseStructure.TIM_Period			= (72000/2-1);      
	TIM_TimeBaseStructure.TIM_ClockDivision		= TIM_CKD_DIV1;
	//TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* Channel 1, 2, 3 and 4 Configuration in Timing mode */
	TIM_OCInitStructure.TIM_OCMode				= TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_Pulse				= 0x0;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	/* Enable the TIM2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel			= TIM3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority	= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd		= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
* @brief 使能按键检测需要使用到的定时器
*/
void Keypad_Timer_Enable(void)
{
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	TIM_Cmd(TIM3, ENABLE);
}

/**
* @brief 关闭按键检测需要使用到的定时器
*/
void Keypad_Timer_Disable(void)
{
	TIM_Cmd(TIM3, DISABLE);
}

/**
* @brief 初始化按键
*/
void Keypad_Init(void)
{
	Keypad_Initport();
	Keypad_Timer_Init();
	//Keypad_Int_Enable();//等待任务运行起来再开按键中断
}


#define KEYPAD_STATE_INIT					0
#define KEYPAD_STATE_AT_LEAST_CLICK			1
#define KEYPAD_STATE_FIRST_CLICK_RELEASE	2
#define KEYPAD_STATE_SECOND_CLICK			3
#define KEYPAD_STATE_LONG_PRESS				4

/**
 * @brief keypad 2个Key对应的IO外部中断ISR
 * @note  EXIT0、EXTI3的中断服务函数调用
*/
void Keypad_EXTI_ISRHandler(void)
{
	if (device_current_state & 0x8000)
	{
		device_current_state &= ~0x8000;
		return;
	}
	//一进入中断，表示检测到有按键被按下，此时需要先关闭所有按键的IO中断，直到此次判断结束再开启
	Keypad_Int_Disable();

	//再开启定时器及其中断，判断按键是单击还是长按
	Keypad_Timer_Enable();

	reset_resVar_ext();
	SCANNER_TRIG_ON();
	//hw_platform_start_led_blink(LED_GREEN,3);
	//scan_key_trig = 1;
	//current_press_key = SCAN_KEY;
	if(device_current_state ==  STATE_BT_Mode_WaitPair || device_current_state ==  STATE_Memory_Mode_BT_WaitPair)
	{
		OSQPost(pEvent_Queue,(void*)EVENT_SCAN_KEY_SINGLE_CLICK);
	}
	press_cnt = 0;
	keypad_state = KEYPAD_STATE_INIT;
}


/**
 * @brief 读取keypad IO，返回IO状态
 * @param[in] unsigned char key		需要读取的是哪个Key对应的IO
 * @return  0: low  else: high  
*/
static unsigned char Keypad_ReadIO(void)
{
	unsigned char h1=0,h2=0;
	unsigned int	i;
reread:
	h1 = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);

	//hw_platform_trip_io();
	for(i=0;i < 10000;i++);	//约1ms(在72M频率下，实测972us)
	//hw_platform_trip_io();

	h2 = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);

	if(h1 == h2)
	{
		return h1;
	}
	else
	{
		goto  reread;
	}
}

//按住超过2个按键低电平判断周期即认为至少是一次按键单击事件的发生，以20ms周期为例，就是只要按住超过40ms即认为至少发生了有效的按键单击事件
#define SINGLE_CLICK_TH			2		
#define LONG_PRESS_TH			300		//按住超过250个按键低电平判断周期即认为是一次按键长按事件的发生，按住超过5S即认为按键长按
#define DOUBLE_CLICK_INTERVAL	4		//双击，连续两次按键之间的时间不超过80ms即认为是双击
//定义一个回调函数指针，以供中断处理函数在获取到按键键值时post给其余模块使用时，可以提供不同的方法
//typedef void (* post_key_method)(unsigned char key_value);

//static void post_key(unsigned char key)
//{
//	OSQPost(pEvent_Queue,(void*)key);
//}

/**
 * @brief 定时器中断处理程序
 * @note 真正判断按键是在20ms定时器中断处理程序完成的
*/
void Keypad_Timer_ISRHandler(void)
{
	if (Keypad_ReadIO() == 0)
	{
		if (keypad_state == KEYPAD_STATE_INIT)
		{
			press_cnt++;
			if (press_cnt == SINGLE_CLICK_TH)
			{
				keypress_timeout = 0;
				keypad_state = KEYPAD_STATE_AT_LEAST_CLICK;
			}
		}
		else if (keypad_state == KEYPAD_STATE_AT_LEAST_CLICK)
		{
			press_cnt++;
			if (press_cnt == LONG_PRESS_TH)
			{
				OSQPost(pEvent_Queue,(void*)EVENT_SCAN_KEY_LONG_PRESS);
				
				//keypad_state = KEYPAD_STATE_INIT;
				//Keypad_Timer_Disable();
				//Keypad_Int_Enable();
			}
		}
		else if (keypad_state == KEYPAD_STATE_FIRST_CLICK_RELEASE)
		{
			keypad_state = KEYPAD_STATE_SECOND_CLICK;
			press_cnt = 0;
		}
		else if (keypad_state == KEYPAD_STATE_SECOND_CLICK)
		{
			press_cnt++;
			if (press_cnt == LONG_PRESS_TH)
			{
				OSQPost(pEvent_Queue,(void*)EVENT_SCAN_KEY_LONG_PRESS);
				//Keypad_Timer_Disable();
				//Keypad_Int_Enable();
			}
		}
	}
	else
	{
		//if((current_press_key == SCAN_KEY)||((current_press_key == ERASE_KEY)&&(device_current_state == STATE_Memory_Mode)))
		{
			SCANNER_TRIG_OFF();
			//scan_key_trig = 0;
			//hw_platform_stop_led_blink(LED_GREEN);
		}

		if (keypad_state == KEYPAD_STATE_INIT)
		{
			//keypad_state = KEYPAD_STATE_INIT;
			Keypad_Timer_Disable();
			Keypad_Int_Enable();
		}
		else if (keypad_state == KEYPAD_STATE_AT_LEAST_CLICK)
		{
			//if (current_press_key == SCAN_KEY)
			{
				//OSQPost(pEvent_Queue,(void*)EVENT_SCAN_KEY_SINGLE_CLICK);
				if (device_current_state == STATE_BT_Mode_Connect)
				{
					//只有在蓝牙连接状态下，才需要检测SCAN键的双击行为
					keypad_state = 	KEYPAD_STATE_FIRST_CLICK_RELEASE;
					release_cnt = 0;
				}
				else
				{
					//其余状态下没有必要检测SCAN键的双击行为
				//	hw_platform_start_led_blink(LED_GREEN,3);
				//	OSQPost(pEvent_Queue,(void*)EVENT_SCAN_KEY_SINGLE_CLICK);
					Keypad_Timer_Disable();
					Keypad_Int_Enable();
				}
				
			}
		}
		else if (keypad_state == KEYPAD_STATE_FIRST_CLICK_RELEASE)
		{
			release_cnt++;
			if (release_cnt == DOUBLE_CLICK_INTERVAL)
			{
				//单击后，在双击间隔时间内没有再次按下按键，及可以确认单击事件的发生
				//hw_platform_start_led_blink(LED_GREEN,3);
				//OSQPost(pEvent_Queue,(void*)EVENT_SCAN_KEY_SINGLE_CLICK);
				Keypad_Timer_Disable();
				Keypad_Int_Enable();
			}
		}
		else if (keypad_state == KEYPAD_STATE_SECOND_CLICK)
		{
			OSQPost(pEvent_Queue,(void*)EVENT_SCAN_KEY_DOUBLE_CLICK);
			//keypad_state = KEYPAD_STATE_INIT;
			Keypad_Timer_Disable();
			Keypad_Int_Enable();
			//hw_platform_stop_led_blink(LED_GREEN);
		}
	}
}

