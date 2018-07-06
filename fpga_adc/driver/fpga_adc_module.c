#ifndef _FPGA_ADC_MODULE_C_
#define _FPGA_ADC_MODULE_C_

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/delay.h>

#include "../../inc/sf_fpga_gp.h"
#include "../../inc/sf_fpga_acp.h"

#include "fpga_adc_module.h"


static int adc_dev_major = 0;
static adc_dev_t adc_dev_info;
static struct tasklet_hrtimer adc_dev_hrtimer;
static acp_pipe_cfg_t adc_rx_pipe;
static adc_fifo_t adc_fifo;

static ktime_t kt;

#if 0
static int dump_packet(unsigned char *buf, int length)
{
	int i;

	printk("\ndump the packet, length: %d:\n", length);
	for(i = 0; i < length; i ++) {
		if(i != 0)
			if(!(i%8))
				printk("\n");
		printk("0x%02x ", buf[i]);
	}
	printk("\n");

	return(RTN_OK);
}
#endif

static ssize_t adc_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *ps = buf;
	int	rs 		= 0;
	int len 	= 0;

	rs = sprintf(ps, "wr_pos:%d rd_pos:%d\n", adc_fifo.wr_pos, adc_fifo.rd_pos);
	len = len + rs;
	ps = ps + rs;
	
	return(len);
}

static ssize_t adc_info_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return ~0;
}

static struct device_attribute dev_attr_adc_info = {
	.attr = {
		.name = "adc_info",
		.mode = S_IWUSR | S_IRUGO,
	},
	.show = adc_info_show,
	.store = adc_info_store,
};

static int adc_rx(u32 data_len, u32 sequ, u32 *pdata)
{
	u32 	wr_pos 		= 0;
	u8 		*pu8_dest 	= NULL;

#if 0
	if(adc_dev_info.open_flag == 0) {
		return(RTN_OK);
	}
#endif

	if(data_len > ADC_DATA_SIZE) {
		printk("Err:Adc frame %d, out of range %d!\n", data_len ,ADC_DATA_SIZE);
		return(-1);
	}
	
	wr_pos = adc_fifo.wr_pos;
		
	pu8_dest = (u8*)&(adc_fifo.adc_fifo_data[wr_pos].adc_data);
	
	memcpy(pu8_dest,pdata,data_len);
	mb();
	
	adc_fifo.adc_fifo_data[wr_pos].data_len = data_len;
	adc_fifo.adc_fifo_data[wr_pos].sequ = sequ;
	
	wr_pos++;
	adc_fifo.wr_pos = (wr_pos & ADC_DATA_FIFO_DEP_MASK);

	return(RTN_OK);
}

static int adc_pipe_rx_check(acp_bd_t *rx_bd, uint32 data_addr)
{	
	u32 	checksum = 0;
	u32 	len 	= 0;
	u32 	i 		= 0;

	if(BD_SN(rx_bd) != DATA_SN(data_addr)) {
		printk("<%d>:%s SN ERROR!\n",__LINE__,__FUNCTION__);
		return(RTN_ERR);
	}

	len = DATA_VAL_LEN(data_addr);
	len = len>>2;  			/*	len/4	*/
	for(i=0; i<len; i++)
		checksum ^= ((uint32 *)data_addr)[i];

	if(checksum != 0)
		return(RTN_ERR);
	
	return(RTN_OK);
}


static int proc_adc_pipe_read(int pipe_no, int rx_bd_cnt, uint32 *data_offset,
						int (*acp_data_callback)(u32 data_len, u32 sequ, u32 *pdata))
{
	s32 proc_bd = 0;
	s32 bd_ps_rd = 0;
	u32 data_len = 0;
	u32 sequ = 0;
	acp_bd_t *rx_bd = NULL;

	u32 data_remap_addr = 0;
	
#define RX_PIPE_PROC_MAX	(256)
	if(rx_bd_cnt>RX_PIPE_PROC_MAX)	rx_bd_cnt = RX_PIPE_PROC_MAX;


	for(proc_bd=0; proc_bd<rx_bd_cnt; proc_bd++) {

		mb();
		bd_ps_rd = adc_rx_pipe.bd_rd + proc_bd;
		rx_bd = (acp_bd_t*)(adc_rx_pipe.bd_vaddr + (bd_ps_rd<<4));

		*data_offset = BD_DATA_ADDR(rx_bd) - adc_rx_pipe.data_baseaddr;
		if(*data_offset >= (adc_rx_pipe.data_size<<5)) {
			printk("<%d>:%s data offset 0x%08x out of range:0x%08x!\n",__LINE__,__FUNCTION__,*data_offset,(adc_rx_pipe.data_size<<5));
		}
		
		data_remap_addr = adc_rx_pipe.data_vaddr + (*data_offset);

		mb();
		if(adc_pipe_rx_check(rx_bd,data_remap_addr) != RTN_OK) {
			continue;
		}
		
		if(DATA_TYPE(data_remap_addr) != DATA_TYPE_ADC)	{
			printk("DATA TYPE ERR!\n");
			continue;
		}	
		
		mb();
		data_len = DATA_VAL_LEN(data_remap_addr);
		sequ = DATA_SN(data_remap_addr);
		acp_data_callback(data_len,sequ,DATA_VAL_DATA_ADC(data_remap_addr));
	}

	bd_ps_rd = (adc_rx_pipe.bd_rd+proc_bd)%adc_rx_pipe.bd_size;
	adc_rx_pipe.bd_rd = bd_ps_rd;
	
	return(proc_bd);
}

static enum hrtimer_restart adc_dev_hrtimer_func(struct hrtimer *timer)
{
	s32 	rx_cnt 		= 0;
	u32 	data_offset = 0;
	s32 	proc_rx_cnt = 0;
	acp_bd_t rx_bd = {0};	/*not used*/
	
	tasklet_hrtimer_start(&adc_dev_hrtimer, kt, HRTIMER_MODE_REL);

	rx_cnt = acp_pipe_read(adc_dev_info.rx_pipe_no,&rx_bd);
	if(rx_cnt != 0) {
		proc_rx_cnt = proc_adc_pipe_read(adc_dev_info.rx_pipe_no,rx_cnt,&data_offset,adc_rx);
		data_offset = data_offset>>5;
		acp_pipe_read_processed(adc_dev_info.rx_pipe_no,proc_rx_cnt,data_offset);
	}

	return (HRTIMER_NORESTART);
}

static int adc_dev_open(struct inode *inode,struct file *file)
{
	return (RTN_OK);
}

static ssize_t adc_dev_read(struct file *file,char __user *buf,size_t count,loff_t *ppos)
{
	s32 	data_cnt 	= 0;
	u8 		*src 		= NULL;
	s32		rs 			= RTN_ERR;
	u32 	rd_pos 		= 0;
	
	adc_dev_t *dev 		= &adc_dev_info;
	adc_data_t *dest 	= (adc_data_t*)buf;
	
	if(adc_fifo.wr_pos == adc_fifo.rd_pos)
		return(0);

	rd_pos = adc_fifo.rd_pos;

	if(adc_fifo.wr_pos >= adc_fifo.rd_pos)
		data_cnt = adc_fifo.wr_pos - rd_pos;
	else
		data_cnt = ADC_DATA_FIFO_DEP - rd_pos;
		
	if (down_trylock(&dev->semRd)) {
		if (file->f_flags & O_NONBLOCK)		return -EAGAIN;
		
		if (down_interruptible(&dev->semRd)) {
			return -ERESTARTSYS;
		}
	}

	if(data_cnt > count)
		data_cnt = count;
	src = (u8*)&(adc_fifo.adc_fifo_data[rd_pos]);

	rs = copy_to_user(dest, src, sizeof(adc_data_t)*data_cnt);
	if(rs != 0) {
		printk("Copy_to_user fail!\n");
		return(0);
	}

	rd_pos = rd_pos + data_cnt;
	
	adc_fifo.rd_pos = (rd_pos & ADC_DATA_FIFO_DEP_MASK);
	
	up(&dev->semRd);
	return(data_cnt);
}

static int adc_dev_mmap(struct file*filp, struct vm_area_struct *vma)
{      
//	vma->vm_flags |= (VM_IO | VM_LOCKED | (VM_DONTEXPAND | VM_DONTDUMP));
     	               
	return(RTN_ERR);
}

static ssize_t adc_dev_write(struct file *file,const char __user *buf,size_t count,loff_t *ppos)
{
	return (RTN_ERR);
}

static unsigned int adc_dev_poll(struct file *file, poll_table *wait)
{
	return (RTN_OK);
}

static long adc_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return (RTN_ERR);
}

static int adc_dev_release(struct inode *inode,struct file *file)
{
	adc_dev_info.open_flag = 0;
	return (RTN_OK);
}

static const struct file_operations adc_dev_ops = {
	.owner = THIS_MODULE,
	.read = adc_dev_read,
	.write = adc_dev_write,
	.open = adc_dev_open,
	.mmap = adc_dev_mmap,
	.poll = adc_dev_poll,
	.unlocked_ioctl = adc_dev_ioctl,
	.release = adc_dev_release,
};

static int acp_mem_map(uint32 phyaddr, uint32 size, uint32 *vaddr, const char *name)
{	
	if (!request_mem_region(phyaddr, size, name)) {
		pr_err("%s(%d) %s request_mem_region fail.\n", __FUNCTION__, __LINE__, name);
			return -EBUSY;
	}

	*vaddr = (uint32)__arm_ioremap(phyaddr, size, MT_DEVICE_CACHED);
	if (!*vaddr) {
		pr_err("%s(%d)ioremap() failed\n", __FUNCTION__, __LINE__);
		release_mem_region(phyaddr, size);
		return -EFAULT;
	}
	
	return(RTN_OK);
}

static void acp_mem_unmap(uint32 phyaddr, uint32 size, uint32 vaddr, const char *name)
{
	iounmap((void *)vaddr);
	release_mem_region(phyaddr, size);
	
	return;
}

static int adc_hw_init(adc_dev_t *dev)
{
	u32 	pipe_rx_no = 0;
	s32		rs = RTN_ERR;
	
	pipe_rx_no = get_rx_pipe(ACP_RX_ADC);
	if(pipe_rx_no == RTN_ERR)
		return(RTN_ERR);
	
	dev->rx_pipe_no = pipe_rx_no;
	dev->rx_pipe = &adc_rx_pipe;

	adc_rx_pipe.pipe_no = pipe_rx_no;
		
	adc_rx_pipe.bd_size = ACP_BD_SIZE;
	adc_rx_pipe.bd_baseaddr = (uint32)alloc_acp_bd_mem(sizeof(acp_bd_t)*ACP_BD_SIZE);
	if((void *)adc_rx_pipe.bd_baseaddr == NULL) {
		printk("<%d>:%s ADC get acp bd mem fail!\n",__LINE__,__FUNCTION__);
	}
	
	adc_rx_pipe.data_size = ACP_DATA_SIZE>>5;	/* aligned (32)*/
	adc_rx_pipe.data_baseaddr = (uint32)alloc_acp_data_mem(ACP_DATA_SIZE);
	if((void *)adc_rx_pipe.data_baseaddr == NULL) {
		printk("<%d>:%s ADC get acp data mem fail!\n",__LINE__,__FUNCTION__);
	}
	
	adc_rx_pipe.bd_rd = 0;
	adc_rx_pipe.bd_wr = 0xFFFFC000;
	adc_rx_pipe.pipe_attr = (DATA_TYPE_ADC<<16)|0xAA;

	rs = acp_mem_map(adc_rx_pipe.bd_baseaddr, sizeof(acp_bd_t)*ACP_BD_SIZE, &(adc_rx_pipe.bd_vaddr),SF_ADC_DEV_NAME);
	if(rs != RTN_OK) {
		printk("<%d>:%s ADC mem map rx bd fail!\n",__LINE__,__FUNCTION__);
		return(rs);
	}
	
	rs = acp_mem_map(adc_rx_pipe.data_baseaddr, ACP_DATA_SIZE, &(adc_rx_pipe.data_vaddr),SF_ADC_DEV_NAME);
	if(rs != RTN_OK) {
		printk("<%d>:%s ADC mem map rx data fail!\n",__LINE__,__FUNCTION__);
		goto mem_map_step1;
	}
	
	adc_rx_pipe.is_valid = ACP_PIPE_VALID;	
	acp_pipe_rx_init(&adc_rx_pipe);

	return(RTN_OK);

mem_map_step1:
	acp_mem_unmap(adc_rx_pipe.bd_baseaddr, adc_rx_pipe.bd_size, adc_rx_pipe.bd_vaddr, SF_ADC_DEV_NAME);	
	
	return(rs);
}

int __init adc_dev_init(void)
{
	s32 	rs 		= RTN_ERR;
	dev_t	dev_id 	= 0;
	struct device* temp = NULL;

	adc_dev_t *dev = &adc_dev_info;

	dev->open_flag = 0;
	rs = alloc_chrdev_region(&dev_id, 0, SF_ADC_DEV_NR, SF_ADC_DEV_NAME);
	adc_dev_major = MAJOR(dev_id);
	if (rs < 0) {
		pr_warn("Adc device can't get major number\n");
		return(RTN_ERR);
	}
	
	cdev_init(&dev->cdev, &adc_dev_ops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &adc_dev_ops;

	rs = cdev_add(&dev->cdev, dev_id, SF_ADC_DEV_NR);
	if (rs < 0)
		goto error_cdev;
	
	dev->adc_dev_class = class_create(THIS_MODULE, SF_ADC_DEV_NAME);
	if (IS_ERR(dev->adc_dev_class)) {
		rs = PTR_ERR(dev->adc_dev_class);
		goto error_cdev;
	}
	
	temp = device_create(dev->adc_dev_class, NULL, dev_id, NULL, SF_ADC_DEV_NAME);
	if(IS_ERR(temp) != 0) {
		rs = PTR_ERR(temp);
		printk(KERN_ALERT"Failed to create device.\n");
		goto destroy_class;
	}

	adc_hw_init(dev);
		
	rs = device_create_file(temp, &dev_attr_adc_info);
	if(rs < 0) {
		printk(KERN_ALERT"Failed to create attribute info device.\n");
		goto destroy_device;
	}

	sema_init(&dev->semRd, 1);
	sema_init(&dev->semWr, 1);
		
	if(rs != RTN_OK)
		goto del_cdev;
	
	kt = ktime_set(0, 500000);

	tasklet_hrtimer_init(&adc_dev_hrtimer, adc_dev_hrtimer_func,
				CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tasklet_hrtimer_start(&adc_dev_hrtimer, kt, HRTIMER_MODE_REL);

	printk("%s COMPILE TIME:%s %s\n",SF_ADC_DEV_NAME,__DATE__,__TIME__);
	printk("VER:%s\n",SF_ADC_VER_INFO);

	return (RTN_OK);
	
destroy_class:
	class_destroy(dev->adc_dev_class);
	
destroy_device:
	device_destroy(dev->adc_dev_class, dev_id);

del_cdev:
	cdev_del(&dev->cdev);

error_cdev:
	unregister_chrdev_region(dev_id, SF_ADC_DEV_NR);

	return(RTN_ERR);
}

void __init adc_dev_exit(void)
{
	dev_t dev_id = MKDEV(adc_dev_major, 0);
	adc_dev_t *dev = &adc_dev_info;
	
	dev->open_flag = 0;

	acp_mem_unmap(adc_rx_pipe.data_baseaddr, ACP_DATA_SIZE, adc_rx_pipe.data_vaddr, SF_ADC_DEV_NAME);	
	acp_mem_unmap(adc_rx_pipe.bd_baseaddr, adc_rx_pipe.bd_size, adc_rx_pipe.bd_vaddr, SF_ADC_DEV_NAME);	
	
	tasklet_hrtimer_cancel(&adc_dev_hrtimer);
	
	device_destroy(dev->adc_dev_class, dev_id);
	class_destroy(dev->adc_dev_class);
	cdev_del(&dev->cdev);
	unregister_chrdev_region(dev_id, SF_ADC_DEV_NR);

	return;
}

module_init(adc_dev_init);
module_exit(adc_dev_exit);

MODULE_AUTHOR("cuijinjin@sf-auto");
MODULE_DESCRIPTION("zynq adc:"SF_ADC_DEV_NAME);
MODULE_LICENSE("Dual BSD/GPL");

#endif /* _FPGA_ADC_MODULE_C_ */
