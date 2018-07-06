CC=arm-xilinx-linux-gnueabi-gcc
LD=arm-xilinx-linux-gnueabi-ld

#CROSS_PATH=$$(dirname $$(which $(CC)))
#LDFLAGS=--sysroot=$(CROSS_PATH)/../arm-none-linux-gnueabi/libc --eh-frame-hdr -dynamic-linker /lib/ld-linux.so.3 -X -m armelf_linux_eabi $(CROSS_PATH)/../arm-none-linux-gnueabi/libc/usr/lib/crt1.o $(CROSS_PATH)/../arm-none-linux-gnueabi/libc/usr/lib/crti.o $(CROSS_PATH)/../lib/gcc/arm-none-linux-gnueabi/4.8.1/crtbegin.o -L$(CROSS_PATH)/../lib/gcc/arm-none-linux-gnueabi/4.8.1 -L$(CROSS_PATH)/../lib/gcc -L$(CROSS_PATH)/../lib/gcc/arm-none-linux-gnueabi/4.8.1/../../../../arm-none-linux-gnueabi/lib -L$(CROSS_PATH)/../arm-none-linux-gnueabi/libc/lib -L$(CROSS_PATH)/../arm-none-linux-gnueabi/libc/usr/lib -lgcc --as-needed -lgcc_s --no-as-needed -lc -lgcc --as-needed -lgcc_s --no-as-needed $(CROSS_PATH)/../lib/gcc/arm-none-linux-gnueabi/4.8.1/crtend.o $(CROSS_PATH)/../arm-none-linux-gnueabi/libc/usr/lib/crtn.o
#$(LD) $(LIBCAN_LDFLAGS) $(LDFLAGS) -o $(TARGET) $(TARGET).o

LDFLAGS= -L./ -lpthread -lsfadc

RUNTIME_PATH=/mnt/sdisk1
#LDFLAGS+=-rpath $(RUNTIME_PATH)

TARGET=adc_test

all:
	$(CC) $(LDFLAGS) -o $(TARGET) main.c
	cp $(TARGET) /samba/nfs/adc/
	cp libsfadc.so /samba/nfs/adc/
clean:
	rm $(TARGET) *.o
