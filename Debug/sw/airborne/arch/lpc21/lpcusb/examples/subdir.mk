################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/lpc21/lpcusb/examples/armVIC.c \
../sw/airborne/arch/lpc21/lpcusb/examples/blockdev_sd.c \
../sw/airborne/arch/lpc21/lpcusb/examples/console.c \
../sw/airborne/arch/lpc21/lpcusb/examples/lpc2000_spi.c \
../sw/airborne/arch/lpc21/lpcusb/examples/lpc2000_spi0.c \
../sw/airborne/arch/lpc21/lpcusb/examples/main_msc.c \
../sw/airborne/arch/lpc21/lpcusb/examples/msc_bot.c \
../sw/airborne/arch/lpc21/lpcusb/examples/msc_scsi.c \
../sw/airborne/arch/lpc21/lpcusb/examples/printf.c \
../sw/airborne/arch/lpc21/lpcusb/examples/startup.c 

OBJS += \
./sw/airborne/arch/lpc21/lpcusb/examples/armVIC.o \
./sw/airborne/arch/lpc21/lpcusb/examples/blockdev_sd.o \
./sw/airborne/arch/lpc21/lpcusb/examples/console.o \
./sw/airborne/arch/lpc21/lpcusb/examples/lpc2000_spi.o \
./sw/airborne/arch/lpc21/lpcusb/examples/lpc2000_spi0.o \
./sw/airborne/arch/lpc21/lpcusb/examples/main_msc.o \
./sw/airborne/arch/lpc21/lpcusb/examples/msc_bot.o \
./sw/airborne/arch/lpc21/lpcusb/examples/msc_scsi.o \
./sw/airborne/arch/lpc21/lpcusb/examples/printf.o \
./sw/airborne/arch/lpc21/lpcusb/examples/startup.o 

C_DEPS += \
./sw/airborne/arch/lpc21/lpcusb/examples/armVIC.d \
./sw/airborne/arch/lpc21/lpcusb/examples/blockdev_sd.d \
./sw/airborne/arch/lpc21/lpcusb/examples/console.d \
./sw/airborne/arch/lpc21/lpcusb/examples/lpc2000_spi.d \
./sw/airborne/arch/lpc21/lpcusb/examples/lpc2000_spi0.d \
./sw/airborne/arch/lpc21/lpcusb/examples/main_msc.d \
./sw/airborne/arch/lpc21/lpcusb/examples/msc_bot.d \
./sw/airborne/arch/lpc21/lpcusb/examples/msc_scsi.d \
./sw/airborne/arch/lpc21/lpcusb/examples/printf.d \
./sw/airborne/arch/lpc21/lpcusb/examples/startup.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/lpc21/lpcusb/examples/%.o: ../sw/airborne/arch/lpc21/lpcusb/examples/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


