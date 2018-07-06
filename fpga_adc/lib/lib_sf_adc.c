#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>

#include <assert.h>

#include "sf_adc.h"
#include "lib_sf_adc.h"
#include "../../ver_inc/git_ver.h"
#include "sflibver.h"

SFLIB_VER_DECLARE(sf_adc, SFADC_LIB_VER, GIT_COMMIT_ID_STR)

static void *gp_global_vbase = NULL;
static void *gp_pwm_vbase = NULL;
static void *gp_adc_vbase = NULL;

#define PWM_DEV_COUNT	(3)
static const unsigned int pwm_tal[PWM_DEV_COUNT] = {GP_PWM0_PHY,GP_PWM1_PHY,GP_PWM2_PHY} ;

static int set_os_mio(int mio_num, unsigned int value)
{
	int  export_fd = 0;
	char pin_des[128];
	int  write_len = 0;
	int  dir_fd = 0, val_fd = 0;

	//! detect exist?
	memset(pin_des, 0, sizeof(pin_des));
	sprintf(pin_des, "/sys/class/gpio/gpio%d", mio_num);
	if(access(pin_des, F_OK))
	{
		//! if not
//		printf("no %s.\n", pin_des);
		export_fd = open("/sys/class/gpio/export", O_WRONLY);
		if (export_fd < 0)
		{
			printf("Cannot open mio[%d] to export it.\n", mio_num);
			return -1;
		}
		memset(pin_des, 0, sizeof(pin_des));
		sprintf(pin_des, "%d", mio_num);
		write_len = strlen(pin_des);
		write_len = write(export_fd, pin_des, write_len);
		if(write_len != strlen(pin_des))
		{
			printf("cannot export mio[%d].\n", mio_num);
			close(export_fd);
			return -1;
		}
		close(export_fd);
	}

	memset(pin_des, 0, sizeof(pin_des));
	sprintf(pin_des, "/sys/class/gpio/gpio%d/direction", mio_num);
	dir_fd = open(pin_des, O_RDWR);
	if (dir_fd < 0)
	{
		printf("cannot open direction mio[%d].\n", mio_num);
		return -1;
	}
	write_len = write(dir_fd, "out", 4);
	if(4 != write_len)
	{
		printf("cannot enable direction mio[%d].\n", mio_num);
		close(dir_fd);
		return -1;
	}
	close(dir_fd);

	memset(pin_des, 0, sizeof(pin_des));
	sprintf(pin_des, "/sys/class/gpio/gpio%d/value", mio_num);
	val_fd = open(pin_des, O_RDWR);
	if (val_fd  <  0)
	{
		printf("Cannot open GPIO value\n");
		return -1;
	}
	memset(pin_des, 0, sizeof(pin_des));
	if(value)
	{
		sprintf(pin_des, "1");
	} else
	{
		sprintf(pin_des, "0");
	}

	write_len = write(val_fd, pin_des, 2);
	if(2 != write_len)
	{
		printf("cannot out mio[%d]--->[%s].\n", mio_num, pin_des);
		close(val_fd);
		return -1;
	}
	close(val_fd);
	return 0;
}

static int init_adc_os(int os_value)
{
	int mem_dev_fd = 0;
	int os_fd = 0;
	int invalid_os = 0;
	unsigned int *ptrFpgaRegADCFlag = 0;
	unsigned int *ptrFpgaRegOSState = 0;

	unsigned int OSValue = 0;
	unsigned int OS0PinValue = 0;
	unsigned int OS1PinValue = 0;
	unsigned int OS2PinValue = 0;
	int OS0PinNum  = 47;
	int OS1PinNum  = 48;
	int OS2PinNum  = 49;

	switch(os_value)
	{
	case 1:
		OS0PinValue = 0;
		OS1PinValue = 0;
		OS2PinValue = 0;
		break;
	case 2:
		OS0PinValue = 1;
		OS1PinValue = 0;
		OS2PinValue = 0;
		break;
	case 4:
		OS0PinValue = 0;
		OS1PinValue = 1;
		OS2PinValue = 0;
		break;
	case 8:
		OS0PinValue = 1;
		OS1PinValue = 1;
		OS2PinValue = 0;
		break;
	case 16:
		OS0PinValue = 0;
		OS1PinValue = 0;
		OS2PinValue = 1;
		break;
	case 32:
		OS0PinValue = 1;
		OS1PinValue = 0;
		OS2PinValue = 1;
		break;
	case 64:
		OS0PinValue = 0;
		OS1PinValue = 1;
		OS2PinValue = 1;
		break;
	default:
		invalid_os = 1;
		break;
	}

	if(invalid_os)
	{
		printf("invalid os value!\n");
		return -1;
	}

	if(set_os_mio(OS0PinNum, OS0PinValue))
	{
		return -1;
	}

	if(set_os_mio(OS1PinNum, OS1PinValue))
	{
	    return -1;
	}

	if(set_os_mio(OS2PinNum, OS2PinValue))
	{
		return -1;
	}

	OSValue  = OS0PinValue;
	OSValue |= OS1PinValue << 1;
	OSValue |= OS2PinValue << 2;

	//!
	mem_dev_fd  =  open(MEM_DEV, O_RDWR);
	assert(mem_dev_fd  >=  0);

	gp_adc_vbase = mmap(NULL, MMAP_SIZE, PROT_READ | PROT_WRITE, \
									MAP_SHARED, mem_dev_fd, GP_ADC_PHY);
	assert(gp_global_vbase  !=  (void *)-1);

	ptrFpgaRegADCFlag = (unsigned int *) gp_adc_vbase;
	if((*ptrFpgaRegADCFlag) != 0xaaaa)
	{
		printf("detect fpga adc module failed!\n");
		munmap(gp_adc_vbase, MMAP_SIZE);
		close(mem_dev_fd);
		gp_adc_vbase = NULL;
		return -1;
	}

	ptrFpgaRegOSState  = (unsigned int *)(gp_adc_vbase + 4);
	*ptrFpgaRegOSState = OSValue;

//	printf("detect fpga adc module flag[0x%08x], OSState[0x%08x].\n",
//			*ptrFpgaRegADCFlag, *ptrFpgaRegOSState);

	munmap(gp_adc_vbase, MMAP_SIZE);
	close(mem_dev_fd);
	gp_adc_vbase = NULL;
	return 0;
}

static int init_pwm(int pwm_no)
{
	int mem_dev_fd = 0;
	fpga_pwm_param *pwm = NULL;
	unsigned int tmp = (2500000 >> 10);
	
	if(pwm_no >= PWM_DEV_COUNT)		return(-1);
	
	mem_dev_fd = open(MEM_DEV, O_RDWR);
	assert(mem_dev_fd >= 0);
	
	gp_global_vbase = mmap(NULL, MMAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_dev_fd, GP_GLOBAL_PHY);
	assert(gp_global_vbase != (void *)-1);
	
	gp_pwm_vbase = mmap(NULL, MMAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_dev_fd, pwm_tal[pwm_no]);
	assert(gp_pwm_vbase != (void *)-1);
	
	pwm = (fpga_pwm_param *)gp_pwm_vbase; 
	pwm->enable = 0x0;
	
	pwm->pos = 40000 + *((unsigned long *)((char *)gp_global_vbase + 0xb8));
	pwm->comp1 = tmp/2;
	pwm->comp2 = tmp - pwm->comp1;
	pwm->rem = tmp & 0x3ff;
	pwm->threshold = 1024;
	pwm->polarity = 0x1;
	
	pwm->enable = 0x1;
	
	munmap(gp_global_vbase, MMAP_SIZE);
		
	close(mem_dev_fd);
	return(0);
}

int sf_adc_open(int os_value)
{
	init_adc_os(os_value);
	init_pwm(2);
	return open(SFADC_DEVICE, O_RDWR | O_NONBLOCK);
}

static void adc_release(void)
{
	fpga_pwm_param *pwm = NULL;

	if(gp_pwm_vbase == NULL)
	{
		printf("adc_pwm_stop: pwm base is null.\n");
		return;
	}

	pwm = (fpga_pwm_param *)gp_pwm_vbase; 

	pwm->enable = 0;

	munmap(gp_pwm_vbase,MMAP_SIZE);
	return;
}

int sf_adc_close(int fd)
{
	adc_release();
	return close(fd);
}

void adc_sample_value_set(unsigned int value)
{
	fpga_pwm_param *pwm = NULL;
	unsigned long tmp = 0;

	if(gp_pwm_vbase  ==  NULL)
	{
		printf("adc_pwm_sample_vale_set: pwm base is null.\n");
		return;
	}
	
	pwm = (fpga_pwm_param *)gp_pwm_vbase; 

	tmp = value>>10;		/* / 1024	*/
	pwm->comp1 = tmp>>1;	/* / 2		*/
	pwm->comp2 = tmp - pwm->comp1;
	pwm->rem = tmp & 0x3ff;	/* ����	*/

	return;
}

