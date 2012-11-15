################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/lpc21/test/bootloader/bootloader.c \
../sw/airborne/arch/lpc21/test/bootloader/console.c \
../sw/airborne/arch/lpc21/test/bootloader/printf.c \
../sw/airborne/arch/lpc21/test/bootloader/startup.c \
../sw/airborne/arch/lpc21/test/bootloader/usbcontrol.c \
../sw/airborne/arch/lpc21/test/bootloader/usbdescrip.c \
../sw/airborne/arch/lpc21/test/bootloader/usbhw_lpc.c \
../sw/airborne/arch/lpc21/test/bootloader/usbinit.c \
../sw/airborne/arch/lpc21/test/bootloader/usbstdreq.c 

S_UPPER_SRCS += \
../sw/airborne/arch/lpc21/test/bootloader/crt.S 

OBJS += \
./sw/airborne/arch/lpc21/test/bootloader/bootloader.o \
./sw/airborne/arch/lpc21/test/bootloader/console.o \
./sw/airborne/arch/lpc21/test/bootloader/crt.o \
./sw/airborne/arch/lpc21/test/bootloader/printf.o \
./sw/airborne/arch/lpc21/test/bootloader/startup.o \
./sw/airborne/arch/lpc21/test/bootloader/usbcontrol.o \
./sw/airborne/arch/lpc21/test/bootloader/usbdescrip.o \
./sw/airborne/arch/lpc21/test/bootloader/usbhw_lpc.o \
./sw/airborne/arch/lpc21/test/bootloader/usbinit.o \
./sw/airborne/arch/lpc21/test/bootloader/usbstdreq.o 

C_DEPS += \
./sw/airborne/arch/lpc21/test/bootloader/bootloader.d \
./sw/airborne/arch/lpc21/test/bootloader/console.d \
./sw/airborne/arch/lpc21/test/bootloader/printf.d \
./sw/airborne/arch/lpc21/test/bootloader/startup.d \
./sw/airborne/arch/lpc21/test/bootloader/usbcontrol.d \
./sw/airborne/arch/lpc21/test/bootloader/usbdescrip.d \
./sw/airborne/arch/lpc21/test/bootloader/usbhw_lpc.d \
./sw/airborne/arch/lpc21/test/bootloader/usbinit.d \
./sw/airborne/arch/lpc21/test/bootloader/usbstdreq.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/lpc21/test/bootloader/%.o: ../sw/airborne/arch/lpc21/test/bootloader/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

sw/airborne/arch/lpc21/test/bootloader/%.o: ../sw/airborne/arch/lpc21/test/bootloader/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


