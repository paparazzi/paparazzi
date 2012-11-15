################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/in_progress/ir_usb_i2c/i2c_usb.c 

OBJS += \
./sw/in_progress/ir_usb_i2c/i2c_usb.o 

C_DEPS += \
./sw/in_progress/ir_usb_i2c/i2c_usb.d 


# Each subdirectory must supply rules for building sources it contributes
sw/in_progress/ir_usb_i2c/%.o: ../sw/in_progress/ir_usb_i2c/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


