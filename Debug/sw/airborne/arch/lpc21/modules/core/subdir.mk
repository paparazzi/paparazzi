################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/lpc21/modules/core/trigger_ext_hw.c 

OBJS += \
./sw/airborne/arch/lpc21/modules/core/trigger_ext_hw.o 

C_DEPS += \
./sw/airborne/arch/lpc21/modules/core/trigger_ext_hw.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/lpc21/modules/core/%.o: ../sw/airborne/arch/lpc21/modules/core/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


