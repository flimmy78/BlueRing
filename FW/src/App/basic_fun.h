#ifndef BASIC_FUN_H
#define BASIC_FUN_H

void itoascii(unsigned int int_value, unsigned char *pValue, unsigned int *plen);
//void int_to_ascii(unsigned int int_value, unsigned char *pValue, unsigned int len, unsigned char type);
unsigned char hex_to_str(unsigned int hex_val, unsigned char type, unsigned char outlen, unsigned char *outstr);
int HexToBCD(unsigned int indata,unsigned char *outdata,unsigned char out_len);
int BCDToHex(unsigned char* indata,unsigned char len,unsigned int *outdata);
unsigned int bcd_field_to_str(unsigned char *bcd_field,unsigned int len,unsigned char *out);
void str_to_bcd_field(unsigned char *str,unsigned char *bcd_field,unsigned int len);
unsigned char HexToAscii(unsigned char hex);
unsigned char Ascii_To_Hex(unsigned char  asciiValue);
int ascii_to_int(unsigned char *pValue);

int	unicode_to_gb2312(unsigned char *pUnicode, unsigned char *pGB2312, int len);

//void ascii_to_keycode(unsigned char *pAscii, unsigned char *pKeycode, unsigned int len);
void ascii_to_keyreport(unsigned char ascii, unsigned char*report);
void ascii_to_keyreport2(unsigned char ascii,unsigned char*report);
unsigned char ascii_to_keyreport2_ext(unsigned char *ascii,unsigned char len,unsigned char*report);

int binary_search(unsigned short *search_list, unsigned int len, unsigned short key);
void build_time_dis_str(unsigned char *date_time_bcd,unsigned char *dis_str,unsigned char mode);

char *F2S(float d, char* str);
int if_last_char_valid(unsigned char *buf,unsigned int len);
int	if_include_unicode_character(unsigned char *buf,unsigned int len);
#endif //BASIC_FUN_H