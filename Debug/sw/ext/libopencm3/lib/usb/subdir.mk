################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ext/libopencm3/lib/usb/usb.c \
../sw/ext/libopencm3/lib/usb/usb_control.c \
../sw/ext/libopencm3/lib/usb/usb_f103.c \
../sw/ext/libopencm3/lib/usb/usb_f107.c \
../sw/ext/libopencm3/lib/usb/usb_f207.c \
../sw/ext/libopencm3/lib/usb/usb_fx07_common.c \
../sw/ext/libopencm3/lib/usb/usb_standard.c 

OBJS += \
./sw/ext/libopencm3/lib/usb/usb.o \
./sw/ext/libopencm3/lib/usb/usb_control.o \
./sw/ext/libopencm3/lib/usb/usb_f103.o \
./sw/ext/libopencm3/lib/usb/usb_f107.o \
./sw/ext/libopencm3/lib/usb/usb_f207.o \
./sw/ext/libopencm3/lib/usb/usb_fx07_common.o \
./sw/ext/libopencm3/lib/usb/usb_standard.o 

C_DEPS += \
./sw/ext/libopencm3/lib/usb/usb.d \
./sw/ext/libopencm3/lib/usb/usb_control.d \
./sw/ext/libopencm3/lib/usb/usb_f103.d \
./sw/ext/libopencm3/lib/usb/usb_f107.d \
./sw/ext/libopencm3/lib/usb/usb_f207.d \
./sw/ext/libopencm3/lib/usb/usb_fx07_common.d \
./sw/ext/libopencm3/lib/usb/usb_standard.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/usb/%.o: ../sw/ext/libopencm3/lib/usb/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


