################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/stm32/subsystems/actuators/actuators_pwm_arch.c 

OBJS += \
./sw/airborne/arch/stm32/subsystems/actuators/actuators_pwm_arch.o 

C_DEPS += \
./sw/airborne/arch/stm32/subsystems/actuators/actuators_pwm_arch.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/stm32/subsystems/actuators/%.o: ../sw/airborne/arch/stm32/subsystems/actuators/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


