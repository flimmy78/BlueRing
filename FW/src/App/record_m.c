/**
 * @file record.c
 * @brief H520B项目脱机记录管理模块
 * @version 1.0
 * @author joe
 * @date 2015年09月28日
 * @note 利用通用的记录管理模块实现的
 * @copy
 *
 * 此代码为深圳合杰电子有限公司项目代码，任何人及公司未经许可不得复制传播，或用于
 * 本公司以外的项目。本司保留一切追究权利。
 *
 * <h1><center>&copy; COPYRIGHT 2015 heroje</center></h1>
 */

#include "record_m.h"
#include "record_mod.h"
#include <string.h>
#include "assert.h"
#include "hw_platform.h"

#define RECORD_MAX_SIZE		(MAX_BARCODE_LEN+1)		//最长的记录大小

#define DELETE_FLAG			0xCF	//记录被删除的标记
unsigned char			rec_module_buffer[RECORD_MAX_SIZE];	//记录模块的公用buffer	

static unsigned int		rec_cnt;

static unsigned int     current_logical_index;
unsigned int             current_node_offset;
unsigned int g_rec_offset;


/**
* @brief  某一个记录区在初始化时需要输入的参数
* @note 如果该记录区的记录需要被修改，那么需要分配record2，否则只需要分配record1即可
*/
typedef struct  
{
	unsigned char	record_index;				//实际使用的记录区索引
	unsigned int	record_size;				//记录的大小
	unsigned int	record_cnt;					//记录的最大条数
} TRec_Init_Param;

//extern  TGOODS_SPEC_NAME_COLLECTION		spec_name;
//extern  TGOODS_SPEC_NAME_COLLECTION		*pSpec_Name_Collection;
/**
 * @brief 系统的所有记录模块的初始化
 * @return 0: OK   else: 错误代码
 * @note: 返回值不能很好的定位到具体的错误发生的位置，后续有需要再修改
 */
int record_module_init(void)
{
	int ret;
	unsigned char flag;

	unsigned int		i;
	//unsigned char		*pRec;


	ret = record_init(1,sizeof(TBATCH_NODE),BATCH_LIST_MAX_CNT);	
	if (ret)
	{
			if (-3 == ret || -6 == ret || -4 == ret)	//记录区还没有Format
			{
				ret = record_format(1,sizeof(TBATCH_NODE),BATCH_LIST_MAX_CNT);
				if (ret)
				{
					//format失败，记录区错误或者没有足够的空间了
					return ret;
				}
			}
			else
			{
				//记录的存储区域有不可恢复的错误
				return ret;
			}
	}	

	rec_cnt = 0;
	for (i = 0;i < record_count_ext(1);i++)
	{
		ret = record_read(0x81,i,&flag,1);
		if (ret)
		{
			return ret;
		}

		if (flag == 0xff)
		{
			//该记录有效
			rec_cnt++;
		}
	}

	if ((rec_cnt == 0)&&(record_count_ext(1) != 0))
	{
		record_delall(1);
	}

	current_logical_index = 0;

	return 0;
}

/**
 * @brief 读指定记录，该记录必须存在，此函数中不作判断
 * @param[in] unsigned char rectype	 记录的类型
 * @param[in] int index 记录的索引  物理索引，该索引对应的记录可能是已经被删除的记录,  1 -- cnt
 * @return 返回记录的地址
 */
unsigned char *record_module_read(unsigned int index)
{
	unsigned char	*pBuf;
	//unsigned int	checkvalue;
	int		ret;
	int		re_read_cnt = 3;		//如果一次读取失败，那么重读3次


	pBuf = rec_module_buffer;

	while(re_read_cnt--)
	{
		ret = record_read(1,record_count_ext(1)-index,pBuf,0);
		if(ret)
		{
			continue;
		}

		return pBuf;
	}

	return (unsigned char*)0;
}

//从start指定的偏移开始搜索第一个被删除的记录
//return -1:		没有搜索到被删除记录，
//		else：  返回被删除的索引
static int get_delete_node(unsigned int start)
{
	unsigned char flag;
	int	i;

	for (i = start; i <= record_count_ext(1);i++)
	{
		if (record_read(0x81,record_count_ext(1)-i,&flag,1))
		{
			return -1;
		}

		if (flag != 0xff)
		{
			return i;
		}
	}

	return -1;
}

//搜索记录区的第一个有效记录
static int get_first_valid_node(void)
{
	unsigned char flag;
	int	i;

	for (i = 1; i <= record_count_ext(1);i++)
	{
		if (record_read(0x81,record_count_ext(1)-i,&flag,1))
		{
			return -1;
		}

		if (flag == 0xff)
		{
			return i;
		}
	}

	return 0;
}


/**
 * @brief 增加一条记录
 * @param[in] unsigned char *precord 记录指针
 * @return 0:成功
 *        -1:记录数已达上限
 * @note 
 */
int record_add(unsigned char *precord)
{
	int			ret;
	int			valid_offset = -1;

	if(record_count_ext(1) >= BATCH_LIST_MAX_CNT)
	{

		if (rec_cnt >= BATCH_LIST_MAX_CNT)
		{
			return -1;
		}
		else
		{
			//找到一个已经被删除的位置
			valid_offset = get_delete_node(1);
			if (valid_offset == -1)
			{
				return -1;
			}
		}
	}


	precord[0] = 0xff;		//保证记录的第一个字节(标志字节)是0xff;
	if (valid_offset != -1)
	{
		//表示新记录需要替换原来被删除的记录
		g_rec_offset = valid_offset;
		ret = record_replace(1, record_count_ext(1)-valid_offset,precord);
	}
	else
	{
		g_rec_offset = record_count_ext(1)+1;
		ret = record_write(1,precord,0);
	}
	
	if(ret)
	{
		return ret;
	}

	rec_cnt++;
	current_node_offset = g_rec_offset;
	return 0;
}



/**
* @brief 往指定记录索引处写一条记录
* @param[in] unsigned char rectype 记录类型
* @param[in] int					记录索引（物理索引）
* @return 0:成功
*        -1:失败
* @note 
*/
int record_module_replace(int index,unsigned char *precord)
{
	precord[0] = 0xff;
	return record_replace(1,record_count_ext(1)-index,precord);
}

/**
 * @brief 得到记录总条数
 * @return 0...LOGIC_RECORD_BLOCK_SIZE
 */
int record_module_count(void)
{
	return rec_cnt;
}

/**
 * @brief 清除所有记录
 * @return none
 */
int record_clear(void)
{
	record_delall(1);
	rec_cnt = 0;  
    return 0;
}

/**
* @brief 根据某一个关键字查询记录
* @param[in] unsigned char *in_tag			要匹配的关键字字符串
* @param[in] int *index						返回逻辑索引
* @return 搜索结果	0:没有搜索到该交易记录  else:搜索到的记录的地址
* @note  实现的时候必须进行字符串比较,从最新的记录开始搜索
*/
unsigned char *rec_searchby_tag(unsigned char *in_tag, int *index)
{
	unsigned char					*pBuf;
	unsigned int					i,led_state;
	int								count,ret;
	unsigned char					flag;

	i = 0;
	led_state = 1;

	count = record_count_ext(1);
	if (count <= 0)
	{
		return (unsigned char*)0;
	}

	//搜索还只能一个个读，一个个比较，不知道当记录太多的时候会不会太慢了？？？
	//顺寻查找方法，需要评估搜索最后一个记录所消耗的时间是否能够忍受，如果不行，那只能通过一些空间换取时间的方法来优化
	//搜索算法了。
	do
	{
		ret = record_read(0x81,i,&flag,1);
		if (ret)
		{
			return (unsigned char*)0;
		}

		if (flag == 0xff)
		{
			pBuf = record_module_read(count-i);

			ret = strcmp((const char*)in_tag,(const char*)((TBATCH_NODE*)pBuf)->barcode);

			if (ret == 0) 
			{
				*index = i;
				//hw_platform_led_ctrl(LED_GREEN,0);
				hw_platform_beep_ctrl(300,3000);
				return pBuf;
			}
		}
		
		i ++;
		//if (i%10 == 0)
		//{
		//	hw_platform_led_ctrl(LED_GREEN,led_state);
		//	led_state = ~led_state;
		//}
		
	}while(i<count);

	//hw_platform_led_ctrl(LED_GREEN,0);
	return (unsigned char*)0;		//没有搜索到与关键字匹配的记录
}


//记录的逻辑偏移对应的物理偏移
int rec_LA_to_PA(unsigned int logical_index)
{
	int ret,i,cnt = 0;
	unsigned char flag;
	unsigned int  num;

	if ((logical_index ==0)||(logical_index > rec_cnt))
	{
		return 0;
	}

	if (logical_index == 1)
	{
		current_logical_index = 1;
		current_node_offset = get_first_valid_node();
		if(logical_index == 1)
		{
			return current_node_offset;
		}
	}

	if (logical_index == current_logical_index)
	{
		return current_node_offset;
	}
	else if (logical_index > current_logical_index)
	{
		num = logical_index - current_logical_index;
		for (i = current_node_offset+1; i<=record_count_ext(1);i++)
		{
			ret = record_read(0x81,record_count_ext(1)-i,&flag,1);
			if (ret)
			{
				return -1;
			}

			if (flag == 0xff)
			{
				cnt++;

				if (cnt == num)
				{
					current_logical_index = logical_index;
					current_node_offset = i;		
					return i;
				}
			}
		}
	}
	else
	{
		num = current_logical_index - logical_index;
		for (i = current_node_offset-1; i > 0;i--)
		{
			ret = record_read(0x81,record_count_ext(1)-i,&flag,1);
			if (ret)
			{
				return -1;
			}

			if (flag == 0xff)
			{
				cnt++;

				if (cnt == num)
				{
					current_logical_index = logical_index;
					current_node_offset = i;		
					return i;
				}
			}
		}

	}
	

	return 0;
}

//记录的物理偏移对应的逻辑偏移
int rec_PA_to_LA(unsigned int physical_index)
{
	int ret,i;//,cnt = 0;
	unsigned char flag;
//	unsigned int num;


	if ((physical_index == 0)||(physical_index > record_count_ext(1)))
	{
		return 0;
	}

	if (physical_index == current_node_offset)
	{
		return current_logical_index;
	}
	else if (physical_index > current_node_offset)
	{
		for (i = current_node_offset+1; i <= physical_index;i++)
		{
			ret = record_read(0x81,record_count_ext(1)-i,&flag,1);
			if (ret)
			{
				return 0;
			}

			if (flag == 0xff)
			{
				current_logical_index++;
			}
		}
	}
	else
	{
		for (i = current_node_offset-1; i >= physical_index;i--)
		{
			ret = record_read(0x81,record_count_ext(1)-i,&flag,1);
			if (ret)
			{
				return 0;
			}

			if (flag == 0xff)
			{
				current_logical_index--;
			}
		}
	}
	current_node_offset = physical_index;	
	return current_logical_index;
}


//清除一个记录
//index  物理偏移
int delete_one_node(unsigned int index)
{
	int ret;
//	unsigned char rec_index;
//	int k;

	ret = record_modify(1,index,0xcf);
	if (ret)
	{
		return ret;
	}

	rec_cnt--;
	return 0;
}


/**
* @brief 逻辑读取记录
* @param[in] unsigned int mode  0:第一个有效节点   1;前一个有效节点   2:后一个有效节点  3:指定节点偏移
* @param[in] unsigned int offset  只有在mode = 3时才有用，物理偏移
*/
unsigned char* get_node(unsigned int mode,unsigned int offset)
{
	unsigned int index;
	//unsigned char flag;

	if (mode == 0)
	{
		index = rec_LA_to_PA(1);
		if (index == 0)
		{
			return (unsigned char *)0;
		}

		return record_module_read(index);
	}
	else if (mode == 1)
	{
		index = rec_LA_to_PA(current_logical_index-1);
		if (index == 0)
		{
			return (unsigned char *)0;
		}

		return record_module_read(index);
	}
	else if (mode == 2)
	{
		index = rec_LA_to_PA(current_logical_index+1);
		if (index == 0)
		{
			return (unsigned char *)0;
		}

		return record_module_read(index);
	}
	else if (mode == 3)
	{
		index = rec_PA_to_LA(offset);
		if (index == 0)
		{
			return (unsigned char *)0;
		}

		return record_module_read(offset);
	}

	return (unsigned char *)0;
}

int recover_record_by_logfile(void)
{
	return 0;
}


unsigned char* record_logical_read(unsigned int logical_index)
{
	int	index;

	index = rec_LA_to_PA(logical_index);
	if (index == 0)
	{
		return (unsigned char*)0;
	}

	return record_module_read(index);
}

void record_m_test(void)
{
	int i,ret;
	unsigned char *pRec;
	TBATCH_NODE		node;
#if 0      
        ret = record_clear();
        assert(ret == 0);
        
	//test record_add() interface
	for (i = 0; i < BATCH_LIST_MAX_CNT;i++)
	{
		sprintf(node.barcode,"test%d",i);
		hw_platform_trip_io();
		ret = record_add((unsigned char*)&node);
		assert(ret == 0);
	}
	

	//test record_module_read() interface
	for (i = 0; i < BATCH_LIST_MAX_CNT;i++)
	{
		hw_platform_trip_io();
		pRec = record_module_read(i+1);
		assert(pRec != 0);
		sprintf(node.barcode,"test%d",i);

		ret = strcmp(((TBATCH_NODE*)pRec)->barcode,node.barcode);
		assert(ret == 0);
	}

	//test rec_searchby_tag()  speed performance			//45000条记录时，搜索最旧的一条记录耗时13.45s
	//此搜索算法的搜索时间是线性的，如果不满足客户需求，需要改进此搜索算法
	hw_platform_trip_io();
	sprintf(node.barcode,"test%d",0);
	pRec = rec_searchby_tag(node.barcode,&i);
	assert(pRec != 0);
	hw_platform_trip_io();

#endif

	for (i = 0; i < BATCH_LIST_MAX_CNT;i++)
	{
		ret = delete_one_node(i+1);
		assert(ret == 0);
		i++;
	}

	for (i = 0; i < record_module_count();i++)
	{
		pRec = get_node((i==0)?0:2,0);
		assert(pRec != 0);
		sprintf(node.barcode,"test%d",2*i+1);

		ret = strcmp(((TBATCH_NODE*)pRec)->barcode,node.barcode);
		assert(ret == 0);
	}
	
}