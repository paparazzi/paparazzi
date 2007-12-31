#
# Makefile
#

APP = i2c_usb

all: $(APP)

clean:
	rm -f $(APP)

$(APP): $(APP).c
	$(CC) -Wall -o $@ $(APP).c -lusb

install:
	install $(APP) $(DESTDIR)/usr/bin
