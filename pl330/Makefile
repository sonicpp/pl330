ARCH=arm
COMPILE=arm-none-eabi-
KERNEL=/media/data0/Dokumenty/Bakalarka/rsoc-framework/buildroot-external/test/zedboard/build/linux-xilinx-v2014.1
MODULE=pl330.ko

$(MODULE): pl330.c pl330.h
	make ARCH=$(ARCH) CROSS_COMPILE=$(COMPILE) -C $(KERNEL) modules M=`pwd`

clean:
	rm -f *.mod.* *.o *.ko
