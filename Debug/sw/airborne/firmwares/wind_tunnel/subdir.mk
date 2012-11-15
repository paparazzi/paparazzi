################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/wind_tunnel/main.c \
../sw/airborne/firmwares/wind_tunnel/main_mb.c \
../sw/airborne/firmwares/wind_tunnel/wt_baro.c \
../sw/airborne/firmwares/wind_tunnel/wt_servo.c 

OBJS += \
./sw/airborne/firmwares/wind_tunnel/main.o \
./sw/airborne/firmwares/wind_tunnel/main_mb.o \
./sw/airborne/firmwares/wind_tunnel/wt_baro.o \
./sw/airborne/firmwares/wind_tunnel/wt_servo.o 

C_DEPS += \
./sw/airborne/firmwares/wind_tunnel/main.d \
./sw/airborne/firmwares/wind_tunnel/main_mb.d \
./sw/airborne/firmwares/wind_tunnel/wt_baro.d \
./sw/airborne/firmwares/wind_tunnel/wt_servo.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/wind_tunnel/%.o: ../sw/airborne/firmwares/wind_tunnel/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


