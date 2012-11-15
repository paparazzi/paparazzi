################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/avr/ant_h_bridge.c \
../sw/airborne/arch/avr/ant_servo.c \
../sw/airborne/arch/avr/ant_spi.c \
../sw/airborne/arch/avr/ant_tracker.c \
../sw/airborne/arch/avr/ant_v2x.c \
../sw/airborne/arch/avr/dc_mc_link.c \
../sw/airborne/arch/avr/dc_mc_power.c \
../sw/airborne/arch/avr/ppm_hw.c \
../sw/airborne/arch/avr/servos_4017.c \
../sw/airborne/arch/avr/servos_direct_hw.c \
../sw/airborne/arch/avr/servos_esc_hw.c \
../sw/airborne/arch/avr/sys_time_hw.c \
../sw/airborne/arch/avr/uart_tunnel.c 

S_UPPER_SRCS += \
../sw/airborne/arch/avr/ahrs_asm.S 

OBJS += \
./sw/airborne/arch/avr/ahrs_asm.o \
./sw/airborne/arch/avr/ant_h_bridge.o \
./sw/airborne/arch/avr/ant_servo.o \
./sw/airborne/arch/avr/ant_spi.o \
./sw/airborne/arch/avr/ant_tracker.o \
./sw/airborne/arch/avr/ant_v2x.o \
./sw/airborne/arch/avr/dc_mc_link.o \
./sw/airborne/arch/avr/dc_mc_power.o \
./sw/airborne/arch/avr/ppm_hw.o \
./sw/airborne/arch/avr/servos_4017.o \
./sw/airborne/arch/avr/servos_direct_hw.o \
./sw/airborne/arch/avr/servos_esc_hw.o \
./sw/airborne/arch/avr/sys_time_hw.o \
./sw/airborne/arch/avr/uart_tunnel.o 

C_DEPS += \
./sw/airborne/arch/avr/ant_h_bridge.d \
./sw/airborne/arch/avr/ant_servo.d \
./sw/airborne/arch/avr/ant_spi.d \
./sw/airborne/arch/avr/ant_tracker.d \
./sw/airborne/arch/avr/ant_v2x.d \
./sw/airborne/arch/avr/dc_mc_link.d \
./sw/airborne/arch/avr/dc_mc_power.d \
./sw/airborne/arch/avr/ppm_hw.d \
./sw/airborne/arch/avr/servos_4017.d \
./sw/airborne/arch/avr/servos_direct_hw.d \
./sw/airborne/arch/avr/servos_esc_hw.d \
./sw/airborne/arch/avr/sys_time_hw.d \
./sw/airborne/arch/avr/uart_tunnel.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/avr/%.o: ../sw/airborne/arch/avr/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

sw/airborne/arch/avr/%.o: ../sw/airborne/arch/avr/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


