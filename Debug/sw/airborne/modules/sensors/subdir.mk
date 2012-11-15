################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/sensors/AOA_adc.c \
../sw/airborne/modules/sensors/airspeed_adc.c \
../sw/airborne/modules/sensors/airspeed_ads1114.c \
../sw/airborne/modules/sensors/airspeed_amsys.c \
../sw/airborne/modules/sensors/airspeed_ets.c \
../sw/airborne/modules/sensors/alt_srf08.c \
../sw/airborne/modules/sensors/baro_MS5534A.c \
../sw/airborne/modules/sensors/baro_amsys.c \
../sw/airborne/modules/sensors/baro_bmp.c \
../sw/airborne/modules/sensors/baro_board_module.c \
../sw/airborne/modules/sensors/baro_ets.c \
../sw/airborne/modules/sensors/baro_hca.c \
../sw/airborne/modules/sensors/baro_ms5611_i2c.c \
../sw/airborne/modules/sensors/baro_scp.c \
../sw/airborne/modules/sensors/baro_scp_i2c.c \
../sw/airborne/modules/sensors/ezcurrent.c \
../sw/airborne/modules/sensors/imu_aspirin2.c \
../sw/airborne/modules/sensors/imu_ppzuav.c \
../sw/airborne/modules/sensors/mag_hmc5843.c \
../sw/airborne/modules/sensors/mag_hmc58xx.c \
../sw/airborne/modules/sensors/mag_micromag_fw.c \
../sw/airborne/modules/sensors/pressure_board_navarro.c \
../sw/airborne/modules/sensors/trigger_ext.c 

OBJS += \
./sw/airborne/modules/sensors/AOA_adc.o \
./sw/airborne/modules/sensors/airspeed_adc.o \
./sw/airborne/modules/sensors/airspeed_ads1114.o \
./sw/airborne/modules/sensors/airspeed_amsys.o \
./sw/airborne/modules/sensors/airspeed_ets.o \
./sw/airborne/modules/sensors/alt_srf08.o \
./sw/airborne/modules/sensors/baro_MS5534A.o \
./sw/airborne/modules/sensors/baro_amsys.o \
./sw/airborne/modules/sensors/baro_bmp.o \
./sw/airborne/modules/sensors/baro_board_module.o \
./sw/airborne/modules/sensors/baro_ets.o \
./sw/airborne/modules/sensors/baro_hca.o \
./sw/airborne/modules/sensors/baro_ms5611_i2c.o \
./sw/airborne/modules/sensors/baro_scp.o \
./sw/airborne/modules/sensors/baro_scp_i2c.o \
./sw/airborne/modules/sensors/ezcurrent.o \
./sw/airborne/modules/sensors/imu_aspirin2.o \
./sw/airborne/modules/sensors/imu_ppzuav.o \
./sw/airborne/modules/sensors/mag_hmc5843.o \
./sw/airborne/modules/sensors/mag_hmc58xx.o \
./sw/airborne/modules/sensors/mag_micromag_fw.o \
./sw/airborne/modules/sensors/pressure_board_navarro.o \
./sw/airborne/modules/sensors/trigger_ext.o 

C_DEPS += \
./sw/airborne/modules/sensors/AOA_adc.d \
./sw/airborne/modules/sensors/airspeed_adc.d \
./sw/airborne/modules/sensors/airspeed_ads1114.d \
./sw/airborne/modules/sensors/airspeed_amsys.d \
./sw/airborne/modules/sensors/airspeed_ets.d \
./sw/airborne/modules/sensors/alt_srf08.d \
./sw/airborne/modules/sensors/baro_MS5534A.d \
./sw/airborne/modules/sensors/baro_amsys.d \
./sw/airborne/modules/sensors/baro_bmp.d \
./sw/airborne/modules/sensors/baro_board_module.d \
./sw/airborne/modules/sensors/baro_ets.d \
./sw/airborne/modules/sensors/baro_hca.d \
./sw/airborne/modules/sensors/baro_ms5611_i2c.d \
./sw/airborne/modules/sensors/baro_scp.d \
./sw/airborne/modules/sensors/baro_scp_i2c.d \
./sw/airborne/modules/sensors/ezcurrent.d \
./sw/airborne/modules/sensors/imu_aspirin2.d \
./sw/airborne/modules/sensors/imu_ppzuav.d \
./sw/airborne/modules/sensors/mag_hmc5843.d \
./sw/airborne/modules/sensors/mag_hmc58xx.d \
./sw/airborne/modules/sensors/mag_micromag_fw.d \
./sw/airborne/modules/sensors/pressure_board_navarro.d \
./sw/airborne/modules/sensors/trigger_ext.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/sensors/%.o: ../sw/airborne/modules/sensors/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


