################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/MPPT/MPPT.c \
../sw/airborne/modules/MPPT/sim_MPPT.c 

OBJS += \
./sw/airborne/modules/MPPT/MPPT.o \
./sw/airborne/modules/MPPT/sim_MPPT.o 

C_DEPS += \
./sw/airborne/modules/MPPT/MPPT.d \
./sw/airborne/modules/MPPT/sim_MPPT.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/MPPT/%.o: ../sw/airborne/modules/MPPT/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


