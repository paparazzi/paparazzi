################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/lpc21/ADS8344.c \
../sw/airborne/arch/lpc21/armVIC.c \
../sw/airborne/arch/lpc21/gpio.c \
../sw/airborne/arch/lpc21/mcu_arch.c \
../sw/airborne/arch/lpc21/micromag_hw.c \
../sw/airborne/arch/lpc21/tacho_mb.c \
../sw/airborne/arch/lpc21/uart_tunnel.c \
../sw/airborne/arch/lpc21/usb_msc_hw.c \
../sw/airborne/arch/lpc21/usb_ser_hw.c \
../sw/airborne/arch/lpc21/usb_tunnel.c 

S_UPPER_SRCS += \
../sw/airborne/arch/lpc21/crt0.S 

OBJS += \
./sw/airborne/arch/lpc21/ADS8344.o \
./sw/airborne/arch/lpc21/armVIC.o \
./sw/airborne/arch/lpc21/crt0.o \
./sw/airborne/arch/lpc21/gpio.o \
./sw/airborne/arch/lpc21/mcu_arch.o \
./sw/airborne/arch/lpc21/micromag_hw.o \
./sw/airborne/arch/lpc21/tacho_mb.o \
./sw/airborne/arch/lpc21/uart_tunnel.o \
./sw/airborne/arch/lpc21/usb_msc_hw.o \
./sw/airborne/arch/lpc21/usb_ser_hw.o \
./sw/airborne/arch/lpc21/usb_tunnel.o 

C_DEPS += \
./sw/airborne/arch/lpc21/ADS8344.d \
./sw/airborne/arch/lpc21/armVIC.d \
./sw/airborne/arch/lpc21/gpio.d \
./sw/airborne/arch/lpc21/mcu_arch.d \
./sw/airborne/arch/lpc21/micromag_hw.d \
./sw/airborne/arch/lpc21/tacho_mb.d \
./sw/airborne/arch/lpc21/uart_tunnel.d \
./sw/airborne/arch/lpc21/usb_msc_hw.d \
./sw/airborne/arch/lpc21/usb_ser_hw.d \
./sw/airborne/arch/lpc21/usb_tunnel.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/lpc21/%.o: ../sw/airborne/arch/lpc21/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

sw/airborne/arch/lpc21/%.o: ../sw/airborne/arch/lpc21/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


