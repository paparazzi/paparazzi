################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ext/libopencm3/examples/stm32/f1/stm32-h103/pwm_6step/pwm_6step.c 

OBJS += \
./sw/ext/libopencm3/examples/stm32/f1/stm32-h103/pwm_6step/pwm_6step.o 

C_DEPS += \
./sw/ext/libopencm3/examples/stm32/f1/stm32-h103/pwm_6step/pwm_6step.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/examples/stm32/f1/stm32-h103/pwm_6step/%.o: ../sw/ext/libopencm3/examples/stm32/f1/stm32-h103/pwm_6step/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


