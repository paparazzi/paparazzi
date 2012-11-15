################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/omap/mcu_periph/uart_arch.c 

OBJS += \
./sw/airborne/arch/omap/mcu_periph/uart_arch.o 

C_DEPS += \
./sw/airborne/arch/omap/mcu_periph/uart_arch.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/omap/mcu_periph/%.o: ../sw/airborne/arch/omap/mcu_periph/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


