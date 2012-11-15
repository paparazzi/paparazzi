################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/subsystems/ahrs/ahrs_aligner.c \
../sw/airborne/subsystems/ahrs/ahrs_float_cmpl.c \
../sw/airborne/subsystems/ahrs/ahrs_float_dcm.c \
../sw/airborne/subsystems/ahrs/ahrs_float_ekf.c \
../sw/airborne/subsystems/ahrs/ahrs_float_lkf.c \
../sw/airborne/subsystems/ahrs/ahrs_infrared.c \
../sw/airborne/subsystems/ahrs/ahrs_int_cmpl_euler.c \
../sw/airborne/subsystems/ahrs/ahrs_int_cmpl_quat.c \
../sw/airborne/subsystems/ahrs/ahrs_sim.c 

OBJS += \
./sw/airborne/subsystems/ahrs/ahrs_aligner.o \
./sw/airborne/subsystems/ahrs/ahrs_float_cmpl.o \
./sw/airborne/subsystems/ahrs/ahrs_float_dcm.o \
./sw/airborne/subsystems/ahrs/ahrs_float_ekf.o \
./sw/airborne/subsystems/ahrs/ahrs_float_lkf.o \
./sw/airborne/subsystems/ahrs/ahrs_infrared.o \
./sw/airborne/subsystems/ahrs/ahrs_int_cmpl_euler.o \
./sw/airborne/subsystems/ahrs/ahrs_int_cmpl_quat.o \
./sw/airborne/subsystems/ahrs/ahrs_sim.o 

C_DEPS += \
./sw/airborne/subsystems/ahrs/ahrs_aligner.d \
./sw/airborne/subsystems/ahrs/ahrs_float_cmpl.d \
./sw/airborne/subsystems/ahrs/ahrs_float_dcm.d \
./sw/airborne/subsystems/ahrs/ahrs_float_ekf.d \
./sw/airborne/subsystems/ahrs/ahrs_float_lkf.d \
./sw/airborne/subsystems/ahrs/ahrs_infrared.d \
./sw/airborne/subsystems/ahrs/ahrs_int_cmpl_euler.d \
./sw/airborne/subsystems/ahrs/ahrs_int_cmpl_quat.d \
./sw/airborne/subsystems/ahrs/ahrs_sim.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/subsystems/ahrs/%.o: ../sw/airborne/subsystems/ahrs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


