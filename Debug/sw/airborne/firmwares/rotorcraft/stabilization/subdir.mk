################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/rotorcraft/stabilization/quat_setpoint_int.c \
../sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_euler_float.c \
../sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_euler_int.c \
../sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_quat_float.c \
../sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.c \
../sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_float.c \
../sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_int.c \
../sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_float.c \
../sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.c \
../sw/airborne/firmwares/rotorcraft/stabilization/stabilization_none.c \
../sw/airborne/firmwares/rotorcraft/stabilization/stabilization_rate.c 

OBJS += \
./sw/airborne/firmwares/rotorcraft/stabilization/quat_setpoint_int.o \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_euler_float.o \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_euler_int.o \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_quat_float.o \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.o \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_float.o \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_int.o \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_float.o \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.o \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_none.o \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_rate.o 

C_DEPS += \
./sw/airborne/firmwares/rotorcraft/stabilization/quat_setpoint_int.d \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_euler_float.d \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_euler_int.d \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_quat_float.d \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.d \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_float.d \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_int.d \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_float.d \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.d \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_none.d \
./sw/airborne/firmwares/rotorcraft/stabilization/stabilization_rate.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/rotorcraft/stabilization/%.o: ../sw/airborne/firmwares/rotorcraft/stabilization/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


