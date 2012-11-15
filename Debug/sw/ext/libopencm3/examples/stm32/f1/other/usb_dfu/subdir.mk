################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ext/libopencm3/examples/stm32/f1/other/usb_dfu/usbdfu.c 

OBJS += \
./sw/ext/libopencm3/examples/stm32/f1/other/usb_dfu/usbdfu.o 

C_DEPS += \
./sw/ext/libopencm3/examples/stm32/f1/other/usb_dfu/usbdfu.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/examples/stm32/f1/other/usb_dfu/%.o: ../sw/ext/libopencm3/examples/stm32/f1/other/usb_dfu/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


