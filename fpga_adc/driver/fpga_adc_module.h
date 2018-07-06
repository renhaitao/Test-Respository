#ifndef _FPGA_ADC_MODULE_H_
#define _FPGA_ADC_MODULE_H_

#include "../../inc/basedefine.h"
#include "../../inc/basetype.h"
#include "../../ver_inc/git_ver.h"

#define SF_ADC_DEV_NR		(1)
#define SF_ADC_DEV_NAME		"sf_adc_dev"
#define SF_ADC_VER_INFO		"0.0.2"

#define ACP_RX_ADC			(0x00)
#define DATA_TYPE_ADC		(3)

#define ACP_BD_SIZE			(0x2000)
#define ACP_DATA_SIZE		(0x400000)

#define ADC_DATA_SIZE		(512)
#define ADC_DATA_FIFO_DEP	(4096)
#define ADC_DATA_FIFO_DEP_MASK	(0xFFF)

typedef struct _adc_data_stru {
	char adc_data[ADC_DATA_SIZE];
	unsigned int data_len;
	unsigned int sequ;
} adc_data_t;

typedef struct _adc_fifo_stru {
	adc_data_t	adc_fifo_data[ADC_DATA_FIFO_DEP];
	unsigned int wr_pos;
	unsigned int rd_pos;
} adc_fifo_t;
	
typedef struct _adc_dev_stru {
	struct cdev cdev;
	struct class *adc_dev_class;
	struct semaphore semRd;
	struct semaphore semWr;
	
	uint32 rx_pipe_no;
	uint32 tx_pipe_no;
	acp_pipe_cfg_t *rx_pipe;
	acp_pipe_cfg_t *tx_pipe;
	uint32 open_flag;
} adc_dev_t;

#define BD_SN(p)			((*((volatile uint32*)p)>>16) & 0x0000FFFF)
#define BD_LEN(p)			((*(volatile uint32*)p) & 0x0000FFFF)
#define BD_DATA_ADDR(p)		((uint32)*((volatile uint32*)p+1))
#define BD_STATE_FLG(p)		((*(((volatile uint32*)p+2))>>28) & 0xF)
#define BD_DATA_TYPE(p)		(*(((volatile uint32*)p+2)) & 0xFF)
#define BD_NET_MASK(p)		(*((volatile uint32*)p+3))

#define DATA_SN(p)			((*((volatile uint32*)p)>>16) & 0x0000FFFF)
#define DATA_TYPE(p)		((*(((volatile uint32*)p+1))>>24) & 0x000000FF)
#define DATA_VAL_LEN(p)		((*((volatile uint32*)p+1)) & 0x0000FFFF)

#define DATA_PORT_INFO(p)	((*(((volatile uint32*)p+2))>>24) & 0x000000FF)
#define DATA_VAL_DATA_ADC(p)	((uint32*)p+2)


#endif /* _FPGA_ADC_MODULE_H_ */