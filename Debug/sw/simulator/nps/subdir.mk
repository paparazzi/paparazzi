################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/simulator/nps/nps_autopilot_rotorcraft.c \
../sw/simulator/nps/nps_fdm_jsbsim.c \
../sw/simulator/nps/nps_flightgear.c \
../sw/simulator/nps/nps_ivy.c \
../sw/simulator/nps/nps_main.c \
../sw/simulator/nps/nps_radio_control.c \
../sw/simulator/nps/nps_radio_control_joystick.c \
../sw/simulator/nps/nps_radio_control_spektrum.c \
../sw/simulator/nps/nps_random.c \
../sw/simulator/nps/nps_sensor_accel.c \
../sw/simulator/nps/nps_sensor_baro.c \
../sw/simulator/nps/nps_sensor_gps.c \
../sw/simulator/nps/nps_sensor_gyro.c \
../sw/simulator/nps/nps_sensor_mag.c \
../sw/simulator/nps/nps_sensors.c \
../sw/simulator/nps/nps_sensors_utils.c 

OBJS += \
./sw/simulator/nps/nps_autopilot_rotorcraft.o \
./sw/simulator/nps/nps_fdm_jsbsim.o \
./sw/simulator/nps/nps_flightgear.o \
./sw/simulator/nps/nps_ivy.o \
./sw/simulator/nps/nps_main.o \
./sw/simulator/nps/nps_radio_control.o \
./sw/simulator/nps/nps_radio_control_joystick.o \
./sw/simulator/nps/nps_radio_control_spektrum.o \
./sw/simulator/nps/nps_random.o \
./sw/simulator/nps/nps_sensor_accel.o \
./sw/simulator/nps/nps_sensor_baro.o \
./sw/simulator/nps/nps_sensor_gps.o \
./sw/simulator/nps/nps_sensor_gyro.o \
./sw/simulator/nps/nps_sensor_mag.o \
./sw/simulator/nps/nps_sensors.o \
./sw/simulator/nps/nps_sensors_utils.o 

C_DEPS += \
./sw/simulator/nps/nps_autopilot_rotorcraft.d \
./sw/simulator/nps/nps_fdm_jsbsim.d \
./sw/simulator/nps/nps_flightgear.d \
./sw/simulator/nps/nps_ivy.d \
./sw/simulator/nps/nps_main.d \
./sw/simulator/nps/nps_radio_control.d \
./sw/simulator/nps/nps_radio_control_joystick.d \
./sw/simulator/nps/nps_radio_control_spektrum.d \
./sw/simulator/nps/nps_random.d \
./sw/simulator/nps/nps_sensor_accel.d \
./sw/simulator/nps/nps_sensor_baro.d \
./sw/simulator/nps/nps_sensor_gps.d \
./sw/simulator/nps/nps_sensor_gyro.d \
./sw/simulator/nps/nps_sensor_mag.d \
./sw/simulator/nps/nps_sensors.d \
./sw/simulator/nps/nps_sensors_utils.d 


# Each subdirectory must supply rules for building sources it contributes
sw/simulator/nps/%.o: ../sw/simulator/nps/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


