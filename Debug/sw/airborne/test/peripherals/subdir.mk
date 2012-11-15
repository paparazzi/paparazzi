################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/test/peripherals/test_ami601.c 

OBJS += \
./sw/airborne/test/peripherals/test_ami601.o 

C_DEPS += \
./sw/airborne/test/peripherals/test_ami601.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/test/peripherals/%.o: ../sw/airborne/test/peripherals/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


