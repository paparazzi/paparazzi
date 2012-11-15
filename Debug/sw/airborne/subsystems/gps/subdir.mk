################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/subsystems/gps/gps_mtk.c \
../sw/airborne/subsystems/gps/gps_nmea.c \
../sw/airborne/subsystems/gps/gps_sim.c \
../sw/airborne/subsystems/gps/gps_sim_nps.c \
../sw/airborne/subsystems/gps/gps_skytraq.c \
../sw/airborne/subsystems/gps/gps_ubx.c 

OBJS += \
./sw/airborne/subsystems/gps/gps_mtk.o \
./sw/airborne/subsystems/gps/gps_nmea.o \
./sw/airborne/subsystems/gps/gps_sim.o \
./sw/airborne/subsystems/gps/gps_sim_nps.o \
./sw/airborne/subsystems/gps/gps_skytraq.o \
./sw/airborne/subsystems/gps/gps_ubx.o 

C_DEPS += \
./sw/airborne/subsystems/gps/gps_mtk.d \
./sw/airborne/subsystems/gps/gps_nmea.d \
./sw/airborne/subsystems/gps/gps_sim.d \
./sw/airborne/subsystems/gps/gps_sim_nps.d \
./sw/airborne/subsystems/gps/gps_skytraq.d \
./sw/airborne/subsystems/gps/gps_ubx.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/subsystems/gps/%.o: ../sw/airborne/subsystems/gps/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


