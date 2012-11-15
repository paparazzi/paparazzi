################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/core/pwm_meas.c \
../sw/airborne/modules/core/sys_mon.c \
../sw/airborne/modules/core/trigger_ext.c 

OBJS += \
./sw/airborne/modules/core/pwm_meas.o \
./sw/airborne/modules/core/sys_mon.o \
./sw/airborne/modules/core/trigger_ext.o 

C_DEPS += \
./sw/airborne/modules/core/pwm_meas.d \
./sw/airborne/modules/core/sys_mon.d \
./sw/airborne/modules/core/trigger_ext.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/core/%.o: ../sw/airborne/modules/core/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


