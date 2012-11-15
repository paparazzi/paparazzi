################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/lpc21/subsystems/actuators/actuators_pwm_arch.c \
../sw/airborne/arch/lpc21/subsystems/actuators/servos_4015_MAT_hw.c \
../sw/airborne/arch/lpc21/subsystems/actuators/servos_4015_hw.c \
../sw/airborne/arch/lpc21/subsystems/actuators/servos_4015_hw_new.c \
../sw/airborne/arch/lpc21/subsystems/actuators/servos_4017_hw.c \
../sw/airborne/arch/lpc21/subsystems/actuators/servos_ppm_hw.c 

OBJS += \
./sw/airborne/arch/lpc21/subsystems/actuators/actuators_pwm_arch.o \
./sw/airborne/arch/lpc21/subsystems/actuators/servos_4015_MAT_hw.o \
./sw/airborne/arch/lpc21/subsystems/actuators/servos_4015_hw.o \
./sw/airborne/arch/lpc21/subsystems/actuators/servos_4015_hw_new.o \
./sw/airborne/arch/lpc21/subsystems/actuators/servos_4017_hw.o \
./sw/airborne/arch/lpc21/subsystems/actuators/servos_ppm_hw.o 

C_DEPS += \
./sw/airborne/arch/lpc21/subsystems/actuators/actuators_pwm_arch.d \
./sw/airborne/arch/lpc21/subsystems/actuators/servos_4015_MAT_hw.d \
./sw/airborne/arch/lpc21/subsystems/actuators/servos_4015_hw.d \
./sw/airborne/arch/lpc21/subsystems/actuators/servos_4015_hw_new.d \
./sw/airborne/arch/lpc21/subsystems/actuators/servos_4017_hw.d \
./sw/airborne/arch/lpc21/subsystems/actuators/servos_ppm_hw.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/lpc21/subsystems/actuators/%.o: ../sw/airborne/arch/lpc21/subsystems/actuators/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


