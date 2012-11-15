################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/lisa/arch/stm32/lisa_overo_link_arch.c 

OBJS += \
./sw/airborne/lisa/arch/stm32/lisa_overo_link_arch.o 

C_DEPS += \
./sw/airborne/lisa/arch/stm32/lisa_overo_link_arch.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/lisa/arch/stm32/%.o: ../sw/airborne/lisa/arch/stm32/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


