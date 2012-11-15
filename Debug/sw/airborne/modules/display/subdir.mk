################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/display/lcd_dogm.c 

OBJS += \
./sw/airborne/modules/display/lcd_dogm.o 

C_DEPS += \
./sw/airborne/modules/display/lcd_dogm.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/display/%.o: ../sw/airborne/modules/display/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


