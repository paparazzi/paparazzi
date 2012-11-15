################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/sim/modules/ins/ins_arduimu.c \
../sw/airborne/arch/sim/modules/ins/ins_arduimu_basic.c 

OBJS += \
./sw/airborne/arch/sim/modules/ins/ins_arduimu.o \
./sw/airborne/arch/sim/modules/ins/ins_arduimu_basic.o 

C_DEPS += \
./sw/airborne/arch/sim/modules/ins/ins_arduimu.d \
./sw/airborne/arch/sim/modules/ins/ins_arduimu_basic.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/sim/modules/ins/%.o: ../sw/airborne/arch/sim/modules/ins/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


