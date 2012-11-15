################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/simulator/old_booz/booz2_sim_main.c \
../sw/simulator/old_booz/booz_flight_model.c \
../sw/simulator/old_booz/booz_flight_model_utils.c \
../sw/simulator/old_booz/booz_flightgear.c \
../sw/simulator/old_booz/booz_joystick.c \
../sw/simulator/old_booz/booz_r250.c \
../sw/simulator/old_booz/booz_randlcg.c \
../sw/simulator/old_booz/booz_sensors_model.c \
../sw/simulator/old_booz/booz_sensors_model_accel.c \
../sw/simulator/old_booz/booz_sensors_model_baro.c \
../sw/simulator/old_booz/booz_sensors_model_gps.c \
../sw/simulator/old_booz/booz_sensors_model_gyro.c \
../sw/simulator/old_booz/booz_sensors_model_mag.c \
../sw/simulator/old_booz/booz_sensors_model_rangemeter.c \
../sw/simulator/old_booz/booz_sensors_model_utils.c \
../sw/simulator/old_booz/booz_wind_model.c 

OBJS += \
./sw/simulator/old_booz/booz2_sim_main.o \
./sw/simulator/old_booz/booz_flight_model.o \
./sw/simulator/old_booz/booz_flight_model_utils.o \
./sw/simulator/old_booz/booz_flightgear.o \
./sw/simulator/old_booz/booz_joystick.o \
./sw/simulator/old_booz/booz_r250.o \
./sw/simulator/old_booz/booz_randlcg.o \
./sw/simulator/old_booz/booz_sensors_model.o \
./sw/simulator/old_booz/booz_sensors_model_accel.o \
./sw/simulator/old_booz/booz_sensors_model_baro.o \
./sw/simulator/old_booz/booz_sensors_model_gps.o \
./sw/simulator/old_booz/booz_sensors_model_gyro.o \
./sw/simulator/old_booz/booz_sensors_model_mag.o \
./sw/simulator/old_booz/booz_sensors_model_rangemeter.o \
./sw/simulator/old_booz/booz_sensors_model_utils.o \
./sw/simulator/old_booz/booz_wind_model.o 

C_DEPS += \
./sw/simulator/old_booz/booz2_sim_main.d \
./sw/simulator/old_booz/booz_flight_model.d \
./sw/simulator/old_booz/booz_flight_model_utils.d \
./sw/simulator/old_booz/booz_flightgear.d \
./sw/simulator/old_booz/booz_joystick.d \
./sw/simulator/old_booz/booz_r250.d \
./sw/simulator/old_booz/booz_randlcg.d \
./sw/simulator/old_booz/booz_sensors_model.d \
./sw/simulator/old_booz/booz_sensors_model_accel.d \
./sw/simulator/old_booz/booz_sensors_model_baro.d \
./sw/simulator/old_booz/booz_sensors_model_gps.d \
./sw/simulator/old_booz/booz_sensors_model_gyro.d \
./sw/simulator/old_booz/booz_sensors_model_mag.d \
./sw/simulator/old_booz/booz_sensors_model_rangemeter.d \
./sw/simulator/old_booz/booz_sensors_model_utils.d \
./sw/simulator/old_booz/booz_wind_model.d 


# Each subdirectory must supply rules for building sources it contributes
sw/simulator/old_booz/%.o: ../sw/simulator/old_booz/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


