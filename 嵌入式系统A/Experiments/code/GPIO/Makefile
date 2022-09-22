KDIR:=/home/shiyan/kernel-for-mceb
CROSS_COMPILE    = arm_v5t_le-
CC    = $(CROSS_COMPILE)gcc
.PHONY: modules clean
obj-m := davinci_dm365_gpios.o
modules:
	make -C $(KDIR) M=`pwd` modules
clean:
	make -C $(KDIR) M=`pwd` modules clean

