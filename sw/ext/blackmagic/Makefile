# Hey Emacs, this is a -*- makefile -*-

BLACKSPHERE = http://blacksphere.co.nz/builds
BLACKMAGIC = blackmagic-20141017.bin

Q=@

all: $(BLACKMAGIC)
	$(Q)../../tools/stm32loader/stm32loader.py -p /dev/ttyACM0 ./$(BLACKMAGIC)

$(BLACKMAGIC):
	@echo "Downloading latest BMP firmware"
	wget -O ./$(BLACKMAGIC) $(BLACKSPHERE)/$(BLACKMAGIC)

clean:
	$(Q) rm -rf ./$(BLACKMAGIC)
