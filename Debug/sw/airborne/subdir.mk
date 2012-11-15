################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/agl_vfilter.c \
../sw/airborne/alt_vfilter.c \
../sw/airborne/inter_mcu.c \
../sw/airborne/joystick.c \
../sw/airborne/link_mcu_spi.c \
../sw/airborne/link_mcu_usart.c \
../sw/airborne/mcu.c \
../sw/airborne/micromag.c \
../sw/airborne/pprz_debug.c \
../sw/airborne/rc_settings.c \
../sw/airborne/state.c 

OBJS += \
./sw/airborne/agl_vfilter.o \
./sw/airborne/alt_vfilter.o \
./sw/airborne/inter_mcu.o \
./sw/airborne/joystick.o \
./sw/airborne/link_mcu_spi.o \
./sw/airborne/link_mcu_usart.o \
./sw/airborne/mcu.o \
./sw/airborne/micromag.o \
./sw/airborne/pprz_debug.o \
./sw/airborne/rc_settings.o \
./sw/airborne/state.o 

C_DEPS += \
./sw/airborne/agl_vfilter.d \
./sw/airborne/alt_vfilter.d \
./sw/airborne/inter_mcu.d \
./sw/airborne/joystick.d \
./sw/airborne/link_mcu_spi.d \
./sw/airborne/link_mcu_usart.d \
./sw/airborne/mcu.d \
./sw/airborne/micromag.d \
./sw/airborne/pprz_debug.d \
./sw/airborne/rc_settings.d \
./sw/airborne/state.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/%.o: ../sw/airborne/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


