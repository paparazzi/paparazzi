################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/in_progress/inertial/C/ahrs_data.c \
../sw/in_progress/inertial/C/ahrs_display.c \
../sw/in_progress/inertial/C/ahrs_euler_ekf.c \
../sw/in_progress/inertial/C/ahrs_quat_ekf.c \
../sw/in_progress/inertial/C/ahrs_quat_fast_ekf.c \
../sw/in_progress/inertial/C/ahrs_quat_fast_ekf_main.c \
../sw/in_progress/inertial/C/ahrs_quat_ukf.c \
../sw/in_progress/inertial/C/ahrs_utils.c \
../sw/in_progress/inertial/C/ekf.c \
../sw/in_progress/inertial/C/linalg.c \
../sw/in_progress/inertial/C/matrix.c \
../sw/in_progress/inertial/C/random.c \
../sw/in_progress/inertial/C/tilt_data.c \
../sw/in_progress/inertial/C/tilt_display.c \
../sw/in_progress/inertial/C/tilt_ekf.c \
../sw/in_progress/inertial/C/tilt_ekf_optim.c \
../sw/in_progress/inertial/C/tilt_fast_ekf.c \
../sw/in_progress/inertial/C/tilt_ukf.c \
../sw/in_progress/inertial/C/tilt_utils.c \
../sw/in_progress/inertial/C/ukf.c 

OBJS += \
./sw/in_progress/inertial/C/ahrs_data.o \
./sw/in_progress/inertial/C/ahrs_display.o \
./sw/in_progress/inertial/C/ahrs_euler_ekf.o \
./sw/in_progress/inertial/C/ahrs_quat_ekf.o \
./sw/in_progress/inertial/C/ahrs_quat_fast_ekf.o \
./sw/in_progress/inertial/C/ahrs_quat_fast_ekf_main.o \
./sw/in_progress/inertial/C/ahrs_quat_ukf.o \
./sw/in_progress/inertial/C/ahrs_utils.o \
./sw/in_progress/inertial/C/ekf.o \
./sw/in_progress/inertial/C/linalg.o \
./sw/in_progress/inertial/C/matrix.o \
./sw/in_progress/inertial/C/random.o \
./sw/in_progress/inertial/C/tilt_data.o \
./sw/in_progress/inertial/C/tilt_display.o \
./sw/in_progress/inertial/C/tilt_ekf.o \
./sw/in_progress/inertial/C/tilt_ekf_optim.o \
./sw/in_progress/inertial/C/tilt_fast_ekf.o \
./sw/in_progress/inertial/C/tilt_ukf.o \
./sw/in_progress/inertial/C/tilt_utils.o \
./sw/in_progress/inertial/C/ukf.o 

C_DEPS += \
./sw/in_progress/inertial/C/ahrs_data.d \
./sw/in_progress/inertial/C/ahrs_display.d \
./sw/in_progress/inertial/C/ahrs_euler_ekf.d \
./sw/in_progress/inertial/C/ahrs_quat_ekf.d \
./sw/in_progress/inertial/C/ahrs_quat_fast_ekf.d \
./sw/in_progress/inertial/C/ahrs_quat_fast_ekf_main.d \
./sw/in_progress/inertial/C/ahrs_quat_ukf.d \
./sw/in_progress/inertial/C/ahrs_utils.d \
./sw/in_progress/inertial/C/ekf.d \
./sw/in_progress/inertial/C/linalg.d \
./sw/in_progress/inertial/C/matrix.d \
./sw/in_progress/inertial/C/random.d \
./sw/in_progress/inertial/C/tilt_data.d \
./sw/in_progress/inertial/C/tilt_display.d \
./sw/in_progress/inertial/C/tilt_ekf.d \
./sw/in_progress/inertial/C/tilt_ekf_optim.d \
./sw/in_progress/inertial/C/tilt_fast_ekf.d \
./sw/in_progress/inertial/C/tilt_ukf.d \
./sw/in_progress/inertial/C/tilt_utils.d \
./sw/in_progress/inertial/C/ukf.d 


# Each subdirectory must supply rules for building sources it contributes
sw/in_progress/inertial/C/%.o: ../sw/in_progress/inertial/C/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


