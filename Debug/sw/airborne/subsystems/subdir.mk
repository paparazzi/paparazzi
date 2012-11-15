################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/subsystems/actuators.c \
../sw/airborne/subsystems/ahrs.c \
../sw/airborne/subsystems/commands.c \
../sw/airborne/subsystems/electrical.c \
../sw/airborne/subsystems/gps.c \
../sw/airborne/subsystems/imu.c \
../sw/airborne/subsystems/ins.c \
../sw/airborne/subsystems/nav.c \
../sw/airborne/subsystems/radio_control.c \
../sw/airborne/subsystems/settings.c 

OBJS += \
./sw/airborne/subsystems/actuators.o \
./sw/airborne/subsystems/ahrs.o \
./sw/airborne/subsystems/commands.o \
./sw/airborne/subsystems/electrical.o \
./sw/airborne/subsystems/gps.o \
./sw/airborne/subsystems/imu.o \
./sw/airborne/subsystems/ins.o \
./sw/airborne/subsystems/nav.o \
./sw/airborne/subsystems/radio_control.o \
./sw/airborne/subsystems/settings.o 

C_DEPS += \
./sw/airborne/subsystems/actuators.d \
./sw/airborne/subsystems/ahrs.d \
./sw/airborne/subsystems/commands.d \
./sw/airborne/subsystems/electrical.d \
./sw/airborne/subsystems/gps.d \
./sw/airborne/subsystems/imu.d \
./sw/airborne/subsystems/ins.d \
./sw/airborne/subsystems/nav.d \
./sw/airborne/subsystems/radio_control.d \
./sw/airborne/subsystems/settings.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/subsystems/%.o: ../sw/airborne/subsystems/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


