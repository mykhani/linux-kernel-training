ifneq ($(KERNELRELEASE),)
obj-m := hello_version.o
else 
KDIR := "/lib/modules/3.13.0-24-generic/build/"

all: 
	$(MAKE) -C $(KDIR) M=$$PWD
endif

