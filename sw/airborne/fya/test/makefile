NAME   = stm_p103_blink_rom

GCC_DIR=/opt/stm32/arm-2009q1/bin

CC      = $(GCC_DIR)/arm-none-eabi-gcc
LD      = $(GCC_DIR)/arm-none-eabi-ld -v
AR      = $(GCC_DIR)/arm-none-eabi-ar
AS      = $(GCC_DIR)/arm-none-eabi-as
CP      = $(GCC_DIR)/arm-none-eabi-objcopy
OD	= $(GCC_DIR)/arm-none-eabi-objdump

CFLAGS  =  -I./ -c -fno-common -O0 -g -mcpu=cortex-m3 -mthumb 
AFLAGS  = -ahls -mapcs-32 -o crt.o
LFLAGS  = -Tstm_p103_blink_rom.cmd -nostartfiles
CPFLAGS = -Obinary
ODFLAGS	= -S

all: test

clean:
	-rm -f main.list main.o main.out main.hex main.map main.bin stm32f10x_rcc.o stm32f10x_gpio.o *~

test: main.out
	@ echo "...copying"
	$(CP) $(CPFLAGS) main.out main.bin
	$(OD) $(ODFLAGS) main.out > main.list

main.out: main.o stm32f10x_rcc.o stm32f10x_gpio.o stm_p103_blink_rom.cmd 
	@ echo "..linking"
	$(LD) $(LFLAGS) -o main.out  main.o stm32f10x_rcc.o stm32f10x_gpio.o 

stm32f10x_rcc.o: stm32f10x_rcc.c 
	@ echo ".compiling"
	 $(CC) $(CFLAGS) stm32f10x_rcc.c 

stm32f10x_gpio.o: stm32f10x_gpio.c
	@ echo ".compiling"
	 $(CC) $(CFLAGS) stm32f10x_gpio.c 

main.o: main.c
	@ echo ".compiling"
	 $(CC) $(CFLAGS) main.c

LOADER=/opt/stm32/stm32loader-a3c51c26ad6c/stm32loader.py
load: test
	$(LOADER) -p /dev/ttyUSB1  -b 115200 -e -w -v main.bin
