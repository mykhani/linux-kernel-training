ifneq ($(KERNELRELEASE),)
obj-m := feserial.o
else
KDIR := $(HOME)/ykhan/linux-3.19.8
all:
	$(MAKE) -C $(KDIR) M=$$PWD
endif
