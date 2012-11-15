################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/boards/navgo/baro_board.c \
../sw/airborne/boards/navgo/imu_navgo.c 

OBJS += \
./sw/airborne/boards/navgo/baro_board.o \
./sw/airborne/boards/navgo/imu_navgo.o 

C_DEPS += \
./sw/airborne/boards/navgo/baro_board.d \
./sw/airborne/boards/navgo/imu_navgo.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/boards/navgo/%.o: ../sw/airborne/boards/navgo/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


