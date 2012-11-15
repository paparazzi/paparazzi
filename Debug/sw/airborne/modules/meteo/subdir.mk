################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/meteo/charge_sens.c \
../sw/airborne/modules/meteo/dust_gp2y.c \
../sw/airborne/modules/meteo/geiger_counter.c \
../sw/airborne/modules/meteo/humid_dpicco.c \
../sw/airborne/modules/meteo/humid_hih.c \
../sw/airborne/modules/meteo/humid_htm_b71.c \
../sw/airborne/modules/meteo/humid_pcap01.c \
../sw/airborne/modules/meteo/humid_sht.c \
../sw/airborne/modules/meteo/humid_sht_i2c.c \
../sw/airborne/modules/meteo/ir_mlx.c \
../sw/airborne/modules/meteo/light_solar.c \
../sw/airborne/modules/meteo/light_temt.c \
../sw/airborne/modules/meteo/temp_lm75.c \
../sw/airborne/modules/meteo/temp_tcouple_adc.c \
../sw/airborne/modules/meteo/temp_temod.c \
../sw/airborne/modules/meteo/temp_tmp102.c \
../sw/airborne/modules/meteo/wind_gfi.c \
../sw/airborne/modules/meteo/windturbine.c 

OBJS += \
./sw/airborne/modules/meteo/charge_sens.o \
./sw/airborne/modules/meteo/dust_gp2y.o \
./sw/airborne/modules/meteo/geiger_counter.o \
./sw/airborne/modules/meteo/humid_dpicco.o \
./sw/airborne/modules/meteo/humid_hih.o \
./sw/airborne/modules/meteo/humid_htm_b71.o \
./sw/airborne/modules/meteo/humid_pcap01.o \
./sw/airborne/modules/meteo/humid_sht.o \
./sw/airborne/modules/meteo/humid_sht_i2c.o \
./sw/airborne/modules/meteo/ir_mlx.o \
./sw/airborne/modules/meteo/light_solar.o \
./sw/airborne/modules/meteo/light_temt.o \
./sw/airborne/modules/meteo/temp_lm75.o \
./sw/airborne/modules/meteo/temp_tcouple_adc.o \
./sw/airborne/modules/meteo/temp_temod.o \
./sw/airborne/modules/meteo/temp_tmp102.o \
./sw/airborne/modules/meteo/wind_gfi.o \
./sw/airborne/modules/meteo/windturbine.o 

C_DEPS += \
./sw/airborne/modules/meteo/charge_sens.d \
./sw/airborne/modules/meteo/dust_gp2y.d \
./sw/airborne/modules/meteo/geiger_counter.d \
./sw/airborne/modules/meteo/humid_dpicco.d \
./sw/airborne/modules/meteo/humid_hih.d \
./sw/airborne/modules/meteo/humid_htm_b71.d \
./sw/airborne/modules/meteo/humid_pcap01.d \
./sw/airborne/modules/meteo/humid_sht.d \
./sw/airborne/modules/meteo/humid_sht_i2c.d \
./sw/airborne/modules/meteo/ir_mlx.d \
./sw/airborne/modules/meteo/light_solar.d \
./sw/airborne/modules/meteo/light_temt.d \
./sw/airborne/modules/meteo/temp_lm75.d \
./sw/airborne/modules/meteo/temp_tcouple_adc.d \
./sw/airborne/modules/meteo/temp_temod.d \
./sw/airborne/modules/meteo/temp_tmp102.d \
./sw/airborne/modules/meteo/wind_gfi.d \
./sw/airborne/modules/meteo/windturbine.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/meteo/%.o: ../sw/airborne/modules/meteo/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


