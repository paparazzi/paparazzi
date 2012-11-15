################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/lisa/lisa_overo_link.c \
../sw/airborne/lisa/lisa_stm_gps_passthrough_main.c \
../sw/airborne/lisa/lisa_stm_passthrough_main.c \
../sw/airborne/lisa/plug_sys.c \
../sw/airborne/lisa/stm_test_spi_link.c \
../sw/airborne/lisa/test_adc.c \
../sw/airborne/lisa/test_csc_servo.c \
../sw/airborne/lisa/test_datalink.c \
../sw/airborne/lisa/test_float.c \
../sw/airborne/lisa/test_led.c \
../sw/airborne/lisa/test_mc.c \
../sw/airborne/lisa/test_periodic.c \
../sw/airborne/lisa/test_servos.c \
../sw/airborne/lisa/test_spi_slave.c \
../sw/airborne/lisa/test_spi_slave2.c \
../sw/airborne/lisa/test_uart_lisal.c \
../sw/airborne/lisa/test_uart_lisam.c \
../sw/airborne/lisa/tunnel_hw.c 

OBJS += \
./sw/airborne/lisa/lisa_overo_link.o \
./sw/airborne/lisa/lisa_stm_gps_passthrough_main.o \
./sw/airborne/lisa/lisa_stm_passthrough_main.o \
./sw/airborne/lisa/plug_sys.o \
./sw/airborne/lisa/stm_test_spi_link.o \
./sw/airborne/lisa/test_adc.o \
./sw/airborne/lisa/test_csc_servo.o \
./sw/airborne/lisa/test_datalink.o \
./sw/airborne/lisa/test_float.o \
./sw/airborne/lisa/test_led.o \
./sw/airborne/lisa/test_mc.o \
./sw/airborne/lisa/test_periodic.o \
./sw/airborne/lisa/test_servos.o \
./sw/airborne/lisa/test_spi_slave.o \
./sw/airborne/lisa/test_spi_slave2.o \
./sw/airborne/lisa/test_uart_lisal.o \
./sw/airborne/lisa/test_uart_lisam.o \
./sw/airborne/lisa/tunnel_hw.o 

C_DEPS += \
./sw/airborne/lisa/lisa_overo_link.d \
./sw/airborne/lisa/lisa_stm_gps_passthrough_main.d \
./sw/airborne/lisa/lisa_stm_passthrough_main.d \
./sw/airborne/lisa/plug_sys.d \
./sw/airborne/lisa/stm_test_spi_link.d \
./sw/airborne/lisa/test_adc.d \
./sw/airborne/lisa/test_csc_servo.d \
./sw/airborne/lisa/test_datalink.d \
./sw/airborne/lisa/test_float.d \
./sw/airborne/lisa/test_led.d \
./sw/airborne/lisa/test_mc.d \
./sw/airborne/lisa/test_periodic.d \
./sw/airborne/lisa/test_servos.d \
./sw/airborne/lisa/test_spi_slave.d \
./sw/airborne/lisa/test_spi_slave2.d \
./sw/airborne/lisa/test_uart_lisal.d \
./sw/airborne/lisa/test_uart_lisam.d \
./sw/airborne/lisa/tunnel_hw.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/lisa/%.o: ../sw/airborne/lisa/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


