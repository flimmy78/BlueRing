/**
 * @file hash.c
 * @brief hash表算法用到的接口
 * @version 1.0
 * @author joe
 * @date 2011年12月21日
 * @copy
 *
 * 此代码为深圳江波龙电子有限公司项目代码，任何人及公司未经许可不得复制传播，或用于
 * 本公司以外的项目。本司保留一切追究权利。
 *
 * <h1><center>&copy; COPYRIGHT 2011 netcom</center></h1>
 */
#include "hash.h"
#include "crc32.h"
#include<string.h>

//static unsigned long cryptTable[0x500];			//5K的内存空间

#if 0
void prepareCryptTable(void)
{ 
    unsigned long seed = 0x00100001, index1 = 0, index2 = 0, i;
	 unsigned long temp1, temp2;
 
    for( index1 = 0; index1 < 0x100; index1++ )
    { 
        for( index2 = index1, i = 0; i < 5; i++, index2 += 0x100 )
        { 
            seed = (seed * 125 + 3) % 0x2AAAAB;
            temp1 = (seed & 0xFFFF) << 0x10;
 
            seed = (seed * 125 + 3) % 0x2AAAAB;
            temp2 = (seed & 0xFFFF);
 
            cryptTable[index2] = ( temp1 | temp2 ); 
       } 
   } 
} 
#endif


/**
* @brief 计算输入字符串的hash值
* @param[in] unsigned char *str				输入字符串指针
* @param[in] unsigned long dwHashType		hash类型 取值 0、1、2
*/
unsigned long HashString(unsigned char *str, unsigned long dwHashType)
{ 
#if 0
    unsigned char *key  = str;
	unsigned long seed1 = 0x7FED7FED;
	unsigned long seed2 = 0xEEEEEEEE;
    int ch;
 
    while( *key != 0 )
    { 
        ch = *key++;
 
        seed1 = cryptTable[(dwHashType << 8) + ch] ^ (seed1 + seed2);
        seed2 = ch + seed1 + seed2 + (seed2 << 5) + 3; 
    }
    return seed1; 
#endif

	//选择CRC的hash算法
	return crc32(dwHashType,str,strlen((char const*)str));
}





