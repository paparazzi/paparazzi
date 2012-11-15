################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/sim/modules/enose/sim_enose.c 

OBJS += \
./sw/airborne/arch/sim/modules/enose/sim_enose.o 

C_DEPS += \
./sw/airborne/arch/sim/modules/enose/sim_enose.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/sim/modules/enose/%.o: ../sw/airborne/arch/sim/modules/enose/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


