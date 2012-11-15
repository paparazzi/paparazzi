################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/test/subsystems/test_ahrs.c \
../sw/airborne/test/subsystems/test_imu.c \
../sw/airborne/test/subsystems/test_radio_control.c \
../sw/airborne/test/subsystems/test_settings.c 

OBJS += \
./sw/airborne/test/subsystems/test_ahrs.o \
./sw/airborne/test/subsystems/test_imu.o \
./sw/airborne/test/subsystems/test_radio_control.o \
./sw/airborne/test/subsystems/test_settings.o 

C_DEPS += \
./sw/airborne/test/subsystems/test_ahrs.d \
./sw/airborne/test/subsystems/test_imu.d \
./sw/airborne/test/subsystems/test_radio_control.d \
./sw/airborne/test/subsystems/test_settings.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/test/subsystems/%.o: ../sw/airborne/test/subsystems/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


