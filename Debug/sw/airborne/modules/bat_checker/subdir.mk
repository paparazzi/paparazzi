################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/bat_checker/bat_checker.c 

OBJS += \
./sw/airborne/modules/bat_checker/bat_checker.o 

C_DEPS += \
./sw/airborne/modules/bat_checker/bat_checker.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/bat_checker/%.o: ../sw/airborne/modules/bat_checker/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


