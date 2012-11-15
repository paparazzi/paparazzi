################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/mcu_periph/can.c \
../sw/airborne/mcu_periph/i2c.c \
../sw/airborne/mcu_periph/pwm_input.c \
../sw/airborne/mcu_periph/spi.c \
../sw/airborne/mcu_periph/sys_time.c \
../sw/airborne/mcu_periph/uart.c 

OBJS += \
./sw/airborne/mcu_periph/can.o \
./sw/airborne/mcu_periph/i2c.o \
./sw/airborne/mcu_periph/pwm_input.o \
./sw/airborne/mcu_periph/spi.o \
./sw/airborne/mcu_periph/sys_time.o \
./sw/airborne/mcu_periph/uart.o 

C_DEPS += \
./sw/airborne/mcu_periph/can.d \
./sw/airborne/mcu_periph/i2c.d \
./sw/airborne/mcu_periph/pwm_input.d \
./sw/airborne/mcu_periph/spi.d \
./sw/airborne/mcu_periph/sys_time.d \
./sw/airborne/mcu_periph/uart.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/mcu_periph/%.o: ../sw/airborne/mcu_periph/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


