#ifndef _HJ5000_SCANNER_H_
#define _HJ5000_SCANNER_H_


int HJ5000_RxISRHandler(unsigned char c);
void scanner_mod_init(void);
void scanner_mod_reset(void);
int scanner_get_barcode(unsigned char *barcode,unsigned int max_num,unsigned char *barcode_type,unsigned int *barcode_len);

#endif