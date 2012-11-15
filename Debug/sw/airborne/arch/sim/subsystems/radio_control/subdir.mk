################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/sim/subsystems/radio_control/ppm_arch.c \
../sw/airborne/arch/sim/subsystems/radio_control/rc_datalink.c 

OBJS += \
./sw/airborne/arch/sim/subsystems/radio_control/ppm_arch.o \
./sw/airborne/arch/sim/subsystems/radio_control/rc_datalink.o 

C_DEPS += \
./sw/airborne/arch/sim/subsystems/radio_control/ppm_arch.d \
./sw/airborne/arch/sim/subsystems/radio_control/rc_datalink.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/sim/subsystems/radio_control/%.o: ../sw/airborne/arch/sim/subsystems/radio_control/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


