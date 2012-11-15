################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/subsystems/actuators/actuators_asctec.c \
../sw/airborne/subsystems/actuators/actuators_mkk.c \
../sw/airborne/subsystems/actuators/actuators_skiron.c \
../sw/airborne/subsystems/actuators/motor_mixing.c 

OBJS += \
./sw/airborne/subsystems/actuators/actuators_asctec.o \
./sw/airborne/subsystems/actuators/actuators_mkk.o \
./sw/airborne/subsystems/actuators/actuators_skiron.o \
./sw/airborne/subsystems/actuators/motor_mixing.o 

C_DEPS += \
./sw/airborne/subsystems/actuators/actuators_asctec.d \
./sw/airborne/subsystems/actuators/actuators_mkk.d \
./sw/airborne/subsystems/actuators/actuators_skiron.d \
./sw/airborne/subsystems/actuators/motor_mixing.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/subsystems/actuators/%.o: ../sw/airborne/subsystems/actuators/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


