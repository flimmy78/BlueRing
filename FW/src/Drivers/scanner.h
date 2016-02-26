#ifndef _SCANNER_H_
#define _SCANNER_H_

//定义不同的扫描引擎
#define SCANNER_EM3096		1
#define SCANNER_UE988		2
#define SCANNER_HJ5000		3

//定义需要发布的版本使用的扫描引擎
//#define   SCANNER		SCANNER_EM3096
//#define   SCANNER		SCANNER_UE988
#define   SCANNER		SCANNER_HJ5000

void scanner_mode_init(unsigned short switch_map);
int scanner_RxISRHandler(unsigned char *c,int len);
int scanner_get_barcode(unsigned char *code_buf, unsigned int inbuf_size,unsigned char *code_type,unsigned int *code_len);
int scanner_get_softVersion(unsigned char *softVer, unsigned char *plen);
void scanner_set_decoder_switch(unsigned short switch_map);
void scanner_reset_param(void);
#endif