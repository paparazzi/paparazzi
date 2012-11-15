################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/test/nova_test_imu.c \
../sw/airborne/test/test_abi.c \
../sw/airborne/test/test_actuators.c \
../sw/airborne/test/test_adcs.c \
../sw/airborne/test/test_algebra.c \
../sw/airborne/test/test_bla.c \
../sw/airborne/test/test_esc_asctecv1_simple.c \
../sw/airborne/test/test_esc_mkk_simple.c \
../sw/airborne/test/test_geodetic.c \
../sw/airborne/test/test_matrix.c \
../sw/airborne/test/test_nav.c \
../sw/airborne/test/test_telemetry.c 

OBJS += \
./sw/airborne/test/nova_test_imu.o \
./sw/airborne/test/test_abi.o \
./sw/airborne/test/test_actuators.o \
./sw/airborne/test/test_adcs.o \
./sw/airborne/test/test_algebra.o \
./sw/airborne/test/test_bla.o \
./sw/airborne/test/test_esc_asctecv1_simple.o \
./sw/airborne/test/test_esc_mkk_simple.o \
./sw/airborne/test/test_geodetic.o \
./sw/airborne/test/test_matrix.o \
./sw/airborne/test/test_nav.o \
./sw/airborne/test/test_telemetry.o 

C_DEPS += \
./sw/airborne/test/nova_test_imu.d \
./sw/airborne/test/test_abi.d \
./sw/airborne/test/test_actuators.d \
./sw/airborne/test/test_adcs.d \
./sw/airborne/test/test_algebra.d \
./sw/airborne/test/test_bla.d \
./sw/airborne/test/test_esc_asctecv1_simple.d \
./sw/airborne/test/test_esc_mkk_simple.d \
./sw/airborne/test/test_geodetic.d \
./sw/airborne/test/test_matrix.d \
./sw/airborne/test/test_nav.d \
./sw/airborne/test/test_telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/test/%.o: ../sw/airborne/test/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


