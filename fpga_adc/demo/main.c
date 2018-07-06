#include <stdio.h>
#include <fcntl.h>
#include <assert.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <unistd.h>
#include <sys/mman.h>

#include "../lib/sf_adc.h"

int dump_mem(unsigned int *buf, int length)
{
	int i;
	int buf_addr = (int)buf;
	int len = length>>2;

	printf("\ndump the mem, length: %d:\n<0x%08x>: ", length,buf_addr);
	
	for(i = 0; i < len; i ++) {
		if(i != 0)
			if(!(i%8)){
				buf_addr = buf_addr+0x20;
				printf("\n<0x%08x>: ",buf_addr);
			}
		printf("0x%08x ", buf[i]);
	}
	printf("\n");

	return 0;
}

static adc_data_t adc_data_buf[4096];

int main(int argc, char **argv)
{
	int fd = 0;
	int count = 0;
	int loop_count = 0;
	int recv_count = 0;
	int	pre_sequ = -1;
	int i = 0;
	
	fd = sf_adc_open(8);
	if(fd <= 0) {
		printf("sf_adc_open error!\n");
	}
//	adc_sample_value_set(20000000);
	while(1) {
		count = sf_adc_read(fd,adc_data_buf,4096);
		
		for(i=0;i<count;i++) {
			/*check sequ*/
			if(pre_sequ != -1 && (adc_data_buf[i].sequ != (pre_sequ+1)&0xFFFF)) {
				printf("0x%08x 0x%08x\n",pre_sequ,adc_data_buf[i].sequ);
			}
				
			pre_sequ = adc_data_buf[i].sequ;
		}	
		
		
		recv_count = recv_count + count;
		
		loop_count++;
		if(loop_count%1000 == 0) {
			
			printf("Get %d frame! 0x%08x\n",recv_count,adc_data_buf[0].sequ);
//			dump_mem((int*)adc_data_buf[0].adc_data, adc_data_buf[0].data_len);
			recv_count = 0;
		}
		usleep(1000);
	}
	
	sf_adc_close(fd);
	
	return(0);
}
