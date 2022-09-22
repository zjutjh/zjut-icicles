KDIR:=/home/shiyan/kernel-for-mceb
CROSS_COMPILE    = arm_v5t_le-
CC    = $(CROSS_COMPILE)gcc
.PHONY: modules clean
obj-m := rtc-x1205.o
modules:
	make -C $(KDIR) M=`pwd` modules
clean:
	make -C $(KDIR) M=`pwd` modules clean
