KERNELDIR := /home/osboxes/imx6ull/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek

CURRENT_PATH := $(shell pwd)

obj-m := leddev.o
target_ko := leddev.ko

dist_ko_path := /home/osboxes/nfs/rootfs/drivers/

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules
clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean

.PHONY:
all: build app
	cp $(target_ko) $(dist_ko_path)
	cp test/led_app.app $(dist_ko_path)


CC_ = /home/osboxes/imx6ull/gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc
app:
	$(CC_) test/led_app.c -o test/led_app.app
