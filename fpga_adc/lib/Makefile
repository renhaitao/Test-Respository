CC=arm-xilinx-linux-gnueabi-gcc
#CFLAGS= -shared -fPIC -DDEBUG
CFLAGS= -shared -fPIC 

all:
	$(CC) $(CFLAGS) -o libsfadc.so lib_sf_adc.c
	cp libsfadc.so ../demo/
clean:
	rm -rf *.so *.out *.out.in *.po
