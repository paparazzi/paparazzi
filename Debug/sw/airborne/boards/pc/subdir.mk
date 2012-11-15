################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/boards/pc/baro_board.c 

OBJS += \
./sw/airborne/boards/pc/baro_board.o 

C_DEPS += \
./sw/airborne/boards/pc/baro_board.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/boards/pc/%.o: ../sw/airborne/boards/pc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


