################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/ins/ahrs_chimu_spi.c \
../sw/airborne/modules/ins/ahrs_chimu_uart.c \
../sw/airborne/modules/ins/alt_filter.c \
../sw/airborne/modules/ins/fw_ins_vn100.c \
../sw/airborne/modules/ins/imu_chimu.c \
../sw/airborne/modules/ins/ins_arduimu.c \
../sw/airborne/modules/ins/ins_arduimu_basic.c \
../sw/airborne/modules/ins/ins_vn100.c \
../sw/airborne/modules/ins/ins_xsens.c 

OBJS += \
./sw/airborne/modules/ins/ahrs_chimu_spi.o \
./sw/airborne/modules/ins/ahrs_chimu_uart.o \
./sw/airborne/modules/ins/alt_filter.o \
./sw/airborne/modules/ins/fw_ins_vn100.o \
./sw/airborne/modules/ins/imu_chimu.o \
./sw/airborne/modules/ins/ins_arduimu.o \
./sw/airborne/modules/ins/ins_arduimu_basic.o \
./sw/airborne/modules/ins/ins_vn100.o \
./sw/airborne/modules/ins/ins_xsens.o 

C_DEPS += \
./sw/airborne/modules/ins/ahrs_chimu_spi.d \
./sw/airborne/modules/ins/ahrs_chimu_uart.d \
./sw/airborne/modules/ins/alt_filter.d \
./sw/airborne/modules/ins/fw_ins_vn100.d \
./sw/airborne/modules/ins/imu_chimu.d \
./sw/airborne/modules/ins/ins_arduimu.d \
./sw/airborne/modules/ins/ins_arduimu_basic.d \
./sw/airborne/modules/ins/ins_vn100.d \
./sw/airborne/modules/ins/ins_xsens.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/ins/%.o: ../sw/airborne/modules/ins/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


