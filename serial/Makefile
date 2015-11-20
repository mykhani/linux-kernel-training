ifneq ($(KERNELRELEASE),)
obj-m := serial.o
else
KDIR := $(HOME)/ykhan/linux-3.19.8
all:
	$(MAKE) -C $(KDIR) M=$$PWD
endif
