#ifndef _KEYPAD_H_
#define _KEYPAD_H_

#define  SCAN_KEY_EXTI_INT		(1<<0)
#define  ERASE_KEY_EXTI_INT		(1<<1)
#define  RESET_KEY_EXTI_INT		(1<<2)


#define  SCAN_KEY		1
#define  ERASE_KEY		2
#define  RESET_KEY		3

		
void Keypad_Init(void);
void Keypad_Int_Disable(void);
void Keypad_Int_Enable(void);
void Keypad_EXTI_ISRHandler(unsigned char	exti_line);
void Keypad_Timer_ISRHandler(void);

#endif
