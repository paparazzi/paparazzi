KERNELSRC = /lib/modules/$(shell uname -r)/build
KSRC = $(shell readlink -f $(KERNELSRC))
FXLOAD = $(wildcard /sbin/fxload) $(wildcard /usr/sbin/fxload) \
	$(wildcard /usr/local/sbin/fxload)
UDEV_RULES = $(wildcard /etc/udev/rules.d/*udev.rules)

ifneq ($(strip $(UDEV_RULES)),)
USE_UDEV = $(shell grep -q RUN $(UDEV_RULES) && echo y)
endif

ifeq ($(KSRC),)
kernel_src_check:
	@echo
	@echo "Unable to find your kernel source tree.  You may need to"
	@echo "install a kernel source package from your distribution vendor."
	@echo
	@echo "(I checked $(KERNELSRC) and nothing was there.)"
	@echo
	@false
endif

ifeq ($(FIRMWARE_DIR),)
FIRMWARE_DIR := $(shell ./checkfwdir)
endif

ifeq ($(FIRMWARE_DIR),)
firmware_dir_check:
	@echo
	@echo "Your hotplug appears to be outdated or installed improperly."
	@echo "Please reinstall hotplug before continuing."
	@echo
	@false
endif

FIRMWARE_DIR_ESCAPED = $(subst /,\\/,$(FIRMWARE_DIR))
FXLOAD_ESCAPED = $(subst /,\\/,$(word 1, $(FXLOAD)))

ifeq ($(FXLOAD_ESCAPED),)
fxload_check:
	@echo
	@echo "Your system appears to be missing fxload.  Please install"
	@echo "fxload before continuing."
	@echo
	@false
endif

all:
	@echo
	@echo '*****' Using kernel source in $(KSRC) '*****'
	@echo
	$(MAKE) modules -C $(KSRC) M=$(shell pwd)/kernel
	sed -e s/@FIRMWARE_DIR@/$(FIRMWARE_DIR_ESCAPED)/ \
			-e s/@FXLOAD@/$(FXLOAD_ESCAPED)/ \
		<hotplug/wis-ezusb.in >hotplug/wis-ezusb
	sed -e s/@FIRMWARE_DIR@/$(FIRMWARE_DIR_ESCAPED)/ \
			-e s/@FXLOAD@/$(FXLOAD_ESCAPED)/ \
		<udev/wis-ezusb.rules.in >udev/wis-ezusb.rules
	$(MAKE) -C apps CFLAGS="-I$(KSRC)/include -I../include"

install:
	$(MAKE) modules_install INSTALL_MOD_PATH=$(DESTDIR) \
		-C $(KSRC) M=$(shell pwd)/kernel
ifeq ($(SKIP_DEPMOD),)
	/sbin/depmod -a
endif
	@echo
	@echo "Installing include files into $(KSRC)/include/linux"
	@echo
	install -m 0644 include/*.h $(DESTDIR)$(KSRC)/include/linux
	@echo
	@echo "Installing firmware files into $(FIRMWARE_DIR)"
	@echo
	[ -d $(DESTDIR)$(FIRMWARE_DIR) ] || \
		install -d $(DESTDIR)$(FIRMWARE_DIR)
	[ -d $(DESTDIR)$(FIRMWARE_DIR)/ezusb ] || \
		install -d $(DESTDIR)$(FIRMWARE_DIR)/ezusb
	rm -f $(DESTDIR)$(FIRMWARE_DIR)/PX-402U.bin
	install -m 0644 firmware/*.bin $(DESTDIR)$(FIRMWARE_DIR)
	install -m 0644 firmware/ezusb/*.hex $(DESTDIR)$(FIRMWARE_DIR)/ezusb
ifeq ($(USE_UDEV),y)
	install -m 0644 udev/wis-ezusb.rules $(DESTDIR)/etc/udev/rules.d
	rm -f $(DESTDIR)/etc/hotplug/usb/wis-ezusb
	rm -f $(DESTDIR)/etc/hotplug/usb/wis.usermap
else
	install -m 0755 hotplug/wis-ezusb $(DESTDIR)/etc/hotplug/usb
	install -m 0644 hotplug/wis.usermap-ezusb \
		$(DESTDIR)/etc/hotplug/usb/wis.usermap
endif
	$(MAKE) install -C apps DESTDIR=$(DESTDIR)

clean:
	$(MAKE) clean -C $(KSRC) M=$(shell pwd)/kernel
	rm -f hotplug/wis-ezusb udev/wis-ezusb.rules
	$(MAKE) clean -C apps
