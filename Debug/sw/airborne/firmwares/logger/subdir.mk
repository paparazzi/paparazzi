################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/logger/main_logger.c 

OBJS += \
./sw/airborne/firmwares/logger/main_logger.o 

C_DEPS += \
./sw/airborne/firmwares/logger/main_logger.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/logger/%.o: ../sw/airborne/firmwares/logger/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


