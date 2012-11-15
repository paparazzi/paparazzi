################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/sim/modules/core/booz_pwm_arch.c \
../sw/airborne/arch/sim/modules/core/trigger_ext_hw.c 

OBJS += \
./sw/airborne/arch/sim/modules/core/booz_pwm_arch.o \
./sw/airborne/arch/sim/modules/core/trigger_ext_hw.o 

C_DEPS += \
./sw/airborne/arch/sim/modules/core/booz_pwm_arch.d \
./sw/airborne/arch/sim/modules/core/trigger_ext_hw.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/sim/modules/core/%.o: ../sw/airborne/arch/sim/modules/core/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


