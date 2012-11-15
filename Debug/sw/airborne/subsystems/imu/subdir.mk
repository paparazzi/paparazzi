################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/subsystems/imu/imu_analog.c \
../sw/airborne/subsystems/imu/imu_aspirin.c \
../sw/airborne/subsystems/imu/imu_aspirin2.c \
../sw/airborne/subsystems/imu/imu_b2.c \
../sw/airborne/subsystems/imu/imu_crista.c \
../sw/airborne/subsystems/imu/imu_dummy.c \
../sw/airborne/subsystems/imu/imu_nps.c 

OBJS += \
./sw/airborne/subsystems/imu/imu_analog.o \
./sw/airborne/subsystems/imu/imu_aspirin.o \
./sw/airborne/subsystems/imu/imu_aspirin2.o \
./sw/airborne/subsystems/imu/imu_b2.o \
./sw/airborne/subsystems/imu/imu_crista.o \
./sw/airborne/subsystems/imu/imu_dummy.o \
./sw/airborne/subsystems/imu/imu_nps.o 

C_DEPS += \
./sw/airborne/subsystems/imu/imu_analog.d \
./sw/airborne/subsystems/imu/imu_aspirin.d \
./sw/airborne/subsystems/imu/imu_aspirin2.d \
./sw/airborne/subsystems/imu/imu_b2.d \
./sw/airborne/subsystems/imu/imu_crista.d \
./sw/airborne/subsystems/imu/imu_dummy.d \
./sw/airborne/subsystems/imu/imu_nps.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/subsystems/imu/%.o: ../sw/airborne/subsystems/imu/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


