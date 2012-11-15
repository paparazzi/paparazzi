################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/motor_bench/main_motor_bench.c \
../sw/airborne/firmwares/motor_bench/main_turntable.c \
../sw/airborne/firmwares/motor_bench/mb_current.c \
../sw/airborne/firmwares/motor_bench/mb_modes.c \
../sw/airborne/firmwares/motor_bench/mb_scale.c \
../sw/airborne/firmwares/motor_bench/mb_servo.c \
../sw/airborne/firmwares/motor_bench/mb_tacho.c \
../sw/airborne/firmwares/motor_bench/mb_twi_controller.c \
../sw/airborne/firmwares/motor_bench/mb_twi_controller_asctech.c \
../sw/airborne/firmwares/motor_bench/mb_twi_controller_mkk.c \
../sw/airborne/firmwares/motor_bench/turntable_systime.c 

OBJS += \
./sw/airborne/firmwares/motor_bench/main_motor_bench.o \
./sw/airborne/firmwares/motor_bench/main_turntable.o \
./sw/airborne/firmwares/motor_bench/mb_current.o \
./sw/airborne/firmwares/motor_bench/mb_modes.o \
./sw/airborne/firmwares/motor_bench/mb_scale.o \
./sw/airborne/firmwares/motor_bench/mb_servo.o \
./sw/airborne/firmwares/motor_bench/mb_tacho.o \
./sw/airborne/firmwares/motor_bench/mb_twi_controller.o \
./sw/airborne/firmwares/motor_bench/mb_twi_controller_asctech.o \
./sw/airborne/firmwares/motor_bench/mb_twi_controller_mkk.o \
./sw/airborne/firmwares/motor_bench/turntable_systime.o 

C_DEPS += \
./sw/airborne/firmwares/motor_bench/main_motor_bench.d \
./sw/airborne/firmwares/motor_bench/main_turntable.d \
./sw/airborne/firmwares/motor_bench/mb_current.d \
./sw/airborne/firmwares/motor_bench/mb_modes.d \
./sw/airborne/firmwares/motor_bench/mb_scale.d \
./sw/airborne/firmwares/motor_bench/mb_servo.d \
./sw/airborne/firmwares/motor_bench/mb_tacho.d \
./sw/airborne/firmwares/motor_bench/mb_twi_controller.d \
./sw/airborne/firmwares/motor_bench/mb_twi_controller_asctech.d \
./sw/airborne/firmwares/motor_bench/mb_twi_controller_mkk.d \
./sw/airborne/firmwares/motor_bench/turntable_systime.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/motor_bench/%.o: ../sw/airborne/firmwares/motor_bench/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


