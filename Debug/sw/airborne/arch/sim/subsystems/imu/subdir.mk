################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/sim/subsystems/imu/imu_aspirin2_arch.c 

OBJS += \
./sw/airborne/arch/sim/subsystems/imu/imu_aspirin2_arch.o 

C_DEPS += \
./sw/airborne/arch/sim/subsystems/imu/imu_aspirin2_arch.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/sim/subsystems/imu/%.o: ../sw/airborne/arch/sim/subsystems/imu/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


