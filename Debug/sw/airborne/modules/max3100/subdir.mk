################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/max3100/max3100_hw.c 

OBJS += \
./sw/airborne/modules/max3100/max3100_hw.o 

C_DEPS += \
./sw/airborne/modules/max3100/max3100_hw.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/max3100/%.o: ../sw/airborne/modules/max3100/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


