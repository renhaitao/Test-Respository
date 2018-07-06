#ifndef SF_SERIAL_H
#define SF_SERIAL_H

#include <unistd.h> 

#define ADC_DATA_SIZE		(512)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _adc_data_stru {
	char adc_data[ADC_DATA_SIZE];
	unsigned int data_len;
	unsigned int sequ;
} adc_data_t;

/*
 *      功能描述：      关闭ADC设备
 *      参数：          fd:ADC设备描述符
 *      返回值：        OK(0)/ERROR(非0)
 */
extern int sf_adc_close(int fd);

/*
 *      功能描述：      读取ADC数据
 *      参数：          fd:ADC设备描述符
 *						buf:数据缓存区
 *						count:缓冲区数量
 *      返回值：        读取数据数量
 */
static inline int sf_adc_read(int fd, adc_data_t* buf, int count)
{
	return read(fd, buf, count);
}

/*
 *      功能描述：      打开ADC设备
 *      参数：          os_value,过采样值,取值范围{1,2,4,8,16,32,64}
 *      返回值：        ADC设备描述符/error(<0)
 */
extern int sf_adc_open(int os_value);

/*
 *      功能描述：      设置采样频率
 *      参数：          valus:每采1024点 tick数量(8ns/tick)
 *      返回值：        无
 */
extern void adc_sample_value_set(unsigned int value);
#ifdef __cplusplus
}
#endif

#endif 

