# Hey Emacs, this is a -*- makefile -*-

BLACKSPHERE = http://blacksphere.co.nz/builds/archive/2014
BLACKMAGIC = blackmagic-20141017.bin

Q=@

all: $(BLACKMAGIC) dfu.py
	$(Q)./stm32_mem.py ./$(BLACKMAGIC)

$(BLACKMAGIC):
	@echo "Downloading latest BMP firmware"
	wget -O ./$(BLACKMAGIC) $(BLACKSPHERE)/$(BLACKMAGIC)

dfu.py:
	ln -s -f ../../tools/dfu/dfu.py dfu.py

clean:
	$(Q) rm -rf ./$(BLACKMAGIC)
