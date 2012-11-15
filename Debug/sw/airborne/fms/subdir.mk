################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/fms/fms_gs_com.c \
../sw/airborne/fms/fms_network.c \
../sw/airborne/fms/fms_periodic.c \
../sw/airborne/fms/fms_serial_port.c \
../sw/airborne/fms/fms_spi_autopilot_msg.c \
../sw/airborne/fms/fms_spi_link.c \
../sw/airborne/fms/fms_spistream_client.c \
../sw/airborne/fms/fms_spistream_daemon.c \
../sw/airborne/fms/fms_test_datalink.c \
../sw/airborne/fms/lpc_test_spi.c \
../sw/airborne/fms/onboard_logger.c \
../sw/airborne/fms/onboard_transport.c \
../sw/airborne/fms/overo_blmc_calibrate.c \
../sw/airborne/fms/overo_test_gps_passthrough.c \
../sw/airborne/fms/overo_test_passthrough.c \
../sw/airborne/fms/overo_test_periodic.c \
../sw/airborne/fms/overo_test_spi_link.c \
../sw/airborne/fms/overo_test_telemetry.c \
../sw/airborne/fms/overo_test_telemetry2.c \
../sw/airborne/fms/test_telemetry.c \
../sw/airborne/fms/test_telemetry_2.c \
../sw/airborne/fms/udp_transport.c \
../sw/airborne/fms/udp_transport2.c 

OBJS += \
./sw/airborne/fms/fms_gs_com.o \
./sw/airborne/fms/fms_network.o \
./sw/airborne/fms/fms_periodic.o \
./sw/airborne/fms/fms_serial_port.o \
./sw/airborne/fms/fms_spi_autopilot_msg.o \
./sw/airborne/fms/fms_spi_link.o \
./sw/airborne/fms/fms_spistream_client.o \
./sw/airborne/fms/fms_spistream_daemon.o \
./sw/airborne/fms/fms_test_datalink.o \
./sw/airborne/fms/lpc_test_spi.o \
./sw/airborne/fms/onboard_logger.o \
./sw/airborne/fms/onboard_transport.o \
./sw/airborne/fms/overo_blmc_calibrate.o \
./sw/airborne/fms/overo_test_gps_passthrough.o \
./sw/airborne/fms/overo_test_passthrough.o \
./sw/airborne/fms/overo_test_periodic.o \
./sw/airborne/fms/overo_test_spi_link.o \
./sw/airborne/fms/overo_test_telemetry.o \
./sw/airborne/fms/overo_test_telemetry2.o \
./sw/airborne/fms/test_telemetry.o \
./sw/airborne/fms/test_telemetry_2.o \
./sw/airborne/fms/udp_transport.o \
./sw/airborne/fms/udp_transport2.o 

C_DEPS += \
./sw/airborne/fms/fms_gs_com.d \
./sw/airborne/fms/fms_network.d \
./sw/airborne/fms/fms_periodic.d \
./sw/airborne/fms/fms_serial_port.d \
./sw/airborne/fms/fms_spi_autopilot_msg.d \
./sw/airborne/fms/fms_spi_link.d \
./sw/airborne/fms/fms_spistream_client.d \
./sw/airborne/fms/fms_spistream_daemon.d \
./sw/airborne/fms/fms_test_datalink.d \
./sw/airborne/fms/lpc_test_spi.d \
./sw/airborne/fms/onboard_logger.d \
./sw/airborne/fms/onboard_transport.d \
./sw/airborne/fms/overo_blmc_calibrate.d \
./sw/airborne/fms/overo_test_gps_passthrough.d \
./sw/airborne/fms/overo_test_passthrough.d \
./sw/airborne/fms/overo_test_periodic.d \
./sw/airborne/fms/overo_test_spi_link.d \
./sw/airborne/fms/overo_test_telemetry.d \
./sw/airborne/fms/overo_test_telemetry2.d \
./sw/airborne/fms/test_telemetry.d \
./sw/airborne/fms/test_telemetry_2.d \
./sw/airborne/fms/udp_transport.d \
./sw/airborne/fms/udp_transport2.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/fms/%.o: ../sw/airborne/fms/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


