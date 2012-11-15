################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/boards/ardrone/baro_board.c \
../sw/airborne/boards/ardrone/test_baro.c 

OBJS += \
./sw/airborne/boards/ardrone/baro_board.o \
./sw/airborne/boards/ardrone/test_baro.o 

C_DEPS += \
./sw/airborne/boards/ardrone/baro_board.d \
./sw/airborne/boards/ardrone/test_baro.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/boards/ardrone/%.o: ../sw/airborne/boards/ardrone/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


