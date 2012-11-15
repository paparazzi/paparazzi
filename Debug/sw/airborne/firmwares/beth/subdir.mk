################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/beth/bench_sensors_can.c \
../sw/airborne/firmwares/beth/bench_sensors_i2c.c \
../sw/airborne/firmwares/beth/main_beth.c \
../sw/airborne/firmwares/beth/main_coders.c \
../sw/airborne/firmwares/beth/main_overo.c \
../sw/airborne/firmwares/beth/main_stm32.c \
../sw/airborne/firmwares/beth/overo_controller.c \
../sw/airborne/firmwares/beth/overo_estimator.c \
../sw/airborne/firmwares/beth/overo_file_logger.c \
../sw/airborne/firmwares/beth/overo_gcs_com.c \
../sw/airborne/firmwares/beth/overo_sfb_controller.c \
../sw/airborne/firmwares/beth/overo_test_uart.c \
../sw/airborne/firmwares/beth/overo_twist_controller.c \
../sw/airborne/firmwares/beth/rcv_telemetry.c \
../sw/airborne/firmwares/beth/uart_hw.c 

OBJS += \
./sw/airborne/firmwares/beth/bench_sensors_can.o \
./sw/airborne/firmwares/beth/bench_sensors_i2c.o \
./sw/airborne/firmwares/beth/main_beth.o \
./sw/airborne/firmwares/beth/main_coders.o \
./sw/airborne/firmwares/beth/main_overo.o \
./sw/airborne/firmwares/beth/main_stm32.o \
./sw/airborne/firmwares/beth/overo_controller.o \
./sw/airborne/firmwares/beth/overo_estimator.o \
./sw/airborne/firmwares/beth/overo_file_logger.o \
./sw/airborne/firmwares/beth/overo_gcs_com.o \
./sw/airborne/firmwares/beth/overo_sfb_controller.o \
./sw/airborne/firmwares/beth/overo_test_uart.o \
./sw/airborne/firmwares/beth/overo_twist_controller.o \
./sw/airborne/firmwares/beth/rcv_telemetry.o \
./sw/airborne/firmwares/beth/uart_hw.o 

C_DEPS += \
./sw/airborne/firmwares/beth/bench_sensors_can.d \
./sw/airborne/firmwares/beth/bench_sensors_i2c.d \
./sw/airborne/firmwares/beth/main_beth.d \
./sw/airborne/firmwares/beth/main_coders.d \
./sw/airborne/firmwares/beth/main_overo.d \
./sw/airborne/firmwares/beth/main_stm32.d \
./sw/airborne/firmwares/beth/overo_controller.d \
./sw/airborne/firmwares/beth/overo_estimator.d \
./sw/airborne/firmwares/beth/overo_file_logger.d \
./sw/airborne/firmwares/beth/overo_gcs_com.d \
./sw/airborne/firmwares/beth/overo_sfb_controller.d \
./sw/airborne/firmwares/beth/overo_test_uart.d \
./sw/airborne/firmwares/beth/overo_twist_controller.d \
./sw/airborne/firmwares/beth/rcv_telemetry.d \
./sw/airborne/firmwares/beth/uart_hw.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/beth/%.o: ../sw/airborne/firmwares/beth/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


