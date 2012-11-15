################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/boards/umarim/baro_board.c \
../sw/airborne/boards/umarim/imu_umarim.c 

OBJS += \
./sw/airborne/boards/umarim/baro_board.o \
./sw/airborne/boards/umarim/imu_umarim.o 

C_DEPS += \
./sw/airborne/boards/umarim/baro_board.d \
./sw/airborne/boards/umarim/imu_umarim.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/boards/umarim/%.o: ../sw/airborne/boards/umarim/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


