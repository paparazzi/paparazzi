################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/rotorcraft/autopilot.c \
../sw/airborne/firmwares/rotorcraft/datalink.c \
../sw/airborne/firmwares/rotorcraft/main.c \
../sw/airborne/firmwares/rotorcraft/navigation.c \
../sw/airborne/firmwares/rotorcraft/stabilization.c \
../sw/airborne/firmwares/rotorcraft/telemetry.c 

OBJS += \
./sw/airborne/firmwares/rotorcraft/autopilot.o \
./sw/airborne/firmwares/rotorcraft/datalink.o \
./sw/airborne/firmwares/rotorcraft/main.o \
./sw/airborne/firmwares/rotorcraft/navigation.o \
./sw/airborne/firmwares/rotorcraft/stabilization.o \
./sw/airborne/firmwares/rotorcraft/telemetry.o 

C_DEPS += \
./sw/airborne/firmwares/rotorcraft/autopilot.d \
./sw/airborne/firmwares/rotorcraft/datalink.d \
./sw/airborne/firmwares/rotorcraft/main.d \
./sw/airborne/firmwares/rotorcraft/navigation.d \
./sw/airborne/firmwares/rotorcraft/stabilization.d \
./sw/airborne/firmwares/rotorcraft/telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/rotorcraft/%.o: ../sw/airborne/firmwares/rotorcraft/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


