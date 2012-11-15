################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/lpc21/modules/sensors/mag_micromag_fw_hw.c \
../sw/airborne/arch/lpc21/modules/sensors/trig_ext_hw.c 

OBJS += \
./sw/airborne/arch/lpc21/modules/sensors/mag_micromag_fw_hw.o \
./sw/airborne/arch/lpc21/modules/sensors/trig_ext_hw.o 

C_DEPS += \
./sw/airborne/arch/lpc21/modules/sensors/mag_micromag_fw_hw.d \
./sw/airborne/arch/lpc21/modules/sensors/trig_ext_hw.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/lpc21/modules/sensors/%.o: ../sw/airborne/arch/lpc21/modules/sensors/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


