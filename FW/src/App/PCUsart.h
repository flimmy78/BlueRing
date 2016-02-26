#ifndef PCUSART_H_
#define PCUSART_H_


#define MASSTORAGE_DEVICE_TYPE_DUMMY_FAT		0	//虚拟文件系统设备
#define MASSTORAGE_DEVICE_TYPE_MSD				1	//MicroSD卡
#define MASSTORAGE_DEVICE_TYPE_SPI_FLASH		2	//SPI_FLASH

typedef struct {
	int						CmdPos;
	unsigned int			DataLength;
	unsigned int			status;
	unsigned char			*CmdBuffer;
	//unsigned char			ResBuffer[PC_COMMAND_LENTH];
}TPCCommand;

extern unsigned char		g_mass_storage_device_type;

void PCUsart_Init(void);
int PCUsart_InByte(unsigned char c);
int PCUsart_process(void);
//void PCUsart_process(void *pp);
//void reset_command(void);
//void init_card_test(void);
void USB_Cable_Config (unsigned char NewState);
void usb_device_init(unsigned char device_type);
#endif //PCUSART_H_