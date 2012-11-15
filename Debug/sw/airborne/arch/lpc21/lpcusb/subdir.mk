################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/airborne/arch/lpc21/lpcusb/usbcontrol.o \
../sw/airborne/arch/lpc21/lpcusb/usbhw_lpc.o \
../sw/airborne/arch/lpc21/lpcusb/usbinit.o \
../sw/airborne/arch/lpc21/lpcusb/usbstdreq.o 

C_SRCS += \
../sw/airborne/arch/lpc21/lpcusb/usbcontrol.c \
../sw/airborne/arch/lpc21/lpcusb/usbhw_lpc.c \
../sw/airborne/arch/lpc21/lpcusb/usbinit.c \
../sw/airborne/arch/lpc21/lpcusb/usbstdreq.c 

OBJS += \
./sw/airborne/arch/lpc21/lpcusb/usbcontrol.o \
./sw/airborne/arch/lpc21/lpcusb/usbhw_lpc.o \
./sw/airborne/arch/lpc21/lpcusb/usbinit.o \
./sw/airborne/arch/lpc21/lpcusb/usbstdreq.o 

C_DEPS += \
./sw/airborne/arch/lpc21/lpcusb/usbcontrol.d \
./sw/airborne/arch/lpc21/lpcusb/usbhw_lpc.d \
./sw/airborne/arch/lpc21/lpcusb/usbinit.d \
./sw/airborne/arch/lpc21/lpcusb/usbstdreq.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/lpc21/lpcusb/%.o: ../sw/airborne/arch/lpc21/lpcusb/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


