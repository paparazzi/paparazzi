################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_arch.old.c \
../sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_arch.very_old.c \
../sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_attempt1.c \
../sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_attempt2_nolib.c \
../sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_attempt3_subtra.c \
../sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_attempt4_nodouble_isr.c 

OBJS += \
./sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_arch.old.o \
./sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_arch.very_old.o \
./sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_attempt1.o \
./sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_attempt2_nolib.o \
./sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_attempt3_subtra.o \
./sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_attempt4_nodouble_isr.o 

C_DEPS += \
./sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_arch.old.d \
./sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_arch.very_old.d \
./sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_attempt1.d \
./sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_attempt2_nolib.d \
./sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_attempt3_subtra.d \
./sw/airborne/arch/ardrone2/mcu_periph/obsolete/i2c_attempt4_nodouble_isr.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/ardrone2/mcu_periph/obsolete/%.o: ../sw/airborne/arch/ardrone2/mcu_periph/obsolete/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


