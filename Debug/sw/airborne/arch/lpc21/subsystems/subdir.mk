################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/lpc21/subsystems/settings_arch.c 

OBJS += \
./sw/airborne/arch/lpc21/subsystems/settings_arch.o 

C_DEPS += \
./sw/airborne/arch/lpc21/subsystems/settings_arch.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/lpc21/subsystems/%.o: ../sw/airborne/arch/lpc21/subsystems/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


