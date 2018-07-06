#ifndef LIB_SF_SERIAL_H
#define LIB_SF_SERIAL_H

#define OK_HEAD                 "\x1B[32m[OK]\033[0m"
#define ER_HEAD                 "\x1B[31m[ER]\033[0m"

#define SFADC_LIB_VER        "0.0.1"

#define PRINT_ERR(fmt,args...)	printf("%s(%d):"fmt,ER_HEAD,__LINE__,##args);
#define PRINT_INFO(fmt,args...) printf("%s(%d):"fmt,OK_HEAD,__LINE__,##args);

#define MEM_DEV              "/dev/mem"
#define SFADC_DEVICE     "/dev/sf_adc_dev"

#define GP_GLOBAL_PHY			(0x40000000)
#define GP_ADC_PHY				(0x402C0000)
#define GP_PWM0_PHY             (0x40E00000)
#define GP_PWM1_PHY				(0x40E40000)
#define GP_PWM2_PHY             (0x40E80000)

#define MMAP_SIZE				(0x1000)

typedef	struct
{
	unsigned long pos;	
	unsigned long comp1;	
	unsigned long comp2;	
	unsigned long rem;	
	unsigned long threshold;	
	unsigned long enable;	
	unsigned long polarity;	
} fpga_pwm_param;
#endif