KERN_DIR = /samba/zynq/linux-xlnx-repo-smp-zheng/output

KBUILD_EXTRA_SYMBOLS += /samba/zynq/driver_zynq_7015/zynq_fpga_driver/gp_module/Module.symvers
KBUILD_EXTRA_SYMBOLS += /samba/zynq/driver_zynq_7015/zynq_fpga_driver/acp_pipe/Module.symvers
#export KBUILD_EXTRA_SYMBOLS

# Comment/uncomment the following line to disable/enable debugging
DEBUG = n

# Add debugging flag (or not) to CFLAGS
ifeq ($(DEBUG), y)
DEBFLAGS = -O -g
else
DEBFLAGS = -O2
endif
CFLAGS_MODULE += $(DEBFLAGS)

TARGET = adc_dev

all:
	make -C $(KERN_DIR) M=`pwd` modules
	cp -f $(TARGET).ko /samba/nfs/adc/
clean:
	rm -rf .tmp_versions modules.order *.o *.ko *.symvers .*.cmd *.mod.c /samba/nfs/adc/$(TARGET).ko

obj-m += $(TARGET).o
$(TARGET)-objs := fpga_adc_module.o
