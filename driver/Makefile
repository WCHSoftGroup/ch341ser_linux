CONFIG_MODULE_SIG=n

KERNELDIR := /lib/modules/$(shell uname -r)/build

PWD :=$(shell pwd)

DRIVERNAME := ch341

obj-m := $(DRIVERNAME).o

# Default target
all:
	$(MAKE) -C $(KERNELDIR)  M=$(PWD) modules
	@echo "=== The target driver file has been generated ==="
	@ls -la *.ko*

# Check default driver compression format
CHECK_KO_XZ := $(shell find /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/ -name "*.ko.xz" 2>/dev/null | head -n1)
CHECK_KO_ZSTD := $(shell find /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/ -name "*.ko.zst" 2>/dev/null | head -n1)

ZSTD := zstd
ZSTD_OPTIONS := -T0

XZ := xz
XZ_OPTIONS := -T0

# Check if the compression tool exists
CHECK_ZSTD := $(shell command -v $(ZSTD) 2> /dev/null)
CHECK_XZ := $(shell command -v $(XZ) 2> /dev/null)

check-tools:
ifndef CHECK_ZSTD
	$(error "Error: Not found zstd tool, please install with: sudo apt install zstd")
endif
ifndef CHECK_XZ
	$(error "Error: Not found xz tool, please install with:: sudo apt install xz-utils")
endif

clean:
	@echo "=== Clean up the build files ==="
	$(MAKE) -C $(KERNELDIR)  M=$(PWD) clean
	rm -rf *.mk .tmp_versions Module.symvers *.mod.c *.o *.ko* .*.cmd Module.markers modules.order *.a *.mod

load:
	@echo "=== Dynamically loaded driver ==="
	insmod $(DRIVERNAME).ko

unload:
	@echo "=== Unloaded driver ==="
	rmmod $(DRIVERNAME)

install: all
	@echo "=== Install driver which support booting with the system... ==="
	insmod $(DRIVERNAME).ko || true
	mkdir -p /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/ || true

ifneq ($(CHECK_KO_XZ),)
	@echo "=== KO_XZ driver install ===" 
	$(XZ) $(XZ_OPTIONS) -q -f -k $(DRIVERNAME).ko
	cp -f $(DRIVERNAME).ko.xz /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/ || true

else ifneq ($(CHECK_KO_ZSTD),)
	@echo "=== KO_ZST driver install ==="
	$(ZSTD) $(ZSTD_OPTIONS) -q -f -k $(DRIVERNAME).ko
	cp -f $(DRIVERNAME).ko.zst /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/ || true

else
	@echo "=== KO driver install ==="
	cp -f $(DRIVERNAME).ko /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/ || true
endif

	depmod -a
	@echo "=== Driver installation completed successfully ==="

uninstall:
	@echo "=== Uninstall driver... ==="
	rmmod $(DRIVERNAME) || true
	rm -rf /lib/modules/$(shell uname -r)/kernel/drivers/usb/serial/$(DRIVERNAME).ko || true
	depmod -a
	@echo "=== Driver uninstallation completed successfully ==="
	
.PHONY: all check-tools load unload install uninstall clean

