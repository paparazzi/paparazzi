################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/sim/ivy_transport.c \
../sw/airborne/arch/sim/jsbsim_ahrs.c \
../sw/airborne/arch/sim/jsbsim_gps.c \
../sw/airborne/arch/sim/jsbsim_hw.c \
../sw/airborne/arch/sim/jsbsim_ir.c \
../sw/airborne/arch/sim/jsbsim_transport.c \
../sw/airborne/arch/sim/led_hw.c \
../sw/airborne/arch/sim/max1167_hw.c \
../sw/airborne/arch/sim/mcu_arch.c \
../sw/airborne/arch/sim/micromag_hw.c \
../sw/airborne/arch/sim/sim_adc_generic.c \
../sw/airborne/arch/sim/sim_ahrs.c \
../sw/airborne/arch/sim/sim_ap.c \
../sw/airborne/arch/sim/sim_baro.c \
../sw/airborne/arch/sim/sim_gps.c \
../sw/airborne/arch/sim/sim_ir.c \
../sw/airborne/arch/sim/sim_uart.c \
../sw/airborne/arch/sim/sim_uart_hw.c 

OBJS += \
./sw/airborne/arch/sim/ivy_transport.o \
./sw/airborne/arch/sim/jsbsim_ahrs.o \
./sw/airborne/arch/sim/jsbsim_gps.o \
./sw/airborne/arch/sim/jsbsim_hw.o \
./sw/airborne/arch/sim/jsbsim_ir.o \
./sw/airborne/arch/sim/jsbsim_transport.o \
./sw/airborne/arch/sim/led_hw.o \
./sw/airborne/arch/sim/max1167_hw.o \
./sw/airborne/arch/sim/mcu_arch.o \
./sw/airborne/arch/sim/micromag_hw.o \
./sw/airborne/arch/sim/sim_adc_generic.o \
./sw/airborne/arch/sim/sim_ahrs.o \
./sw/airborne/arch/sim/sim_ap.o \
./sw/airborne/arch/sim/sim_baro.o \
./sw/airborne/arch/sim/sim_gps.o \
./sw/airborne/arch/sim/sim_ir.o \
./sw/airborne/arch/sim/sim_uart.o \
./sw/airborne/arch/sim/sim_uart_hw.o 

C_DEPS += \
./sw/airborne/arch/sim/ivy_transport.d \
./sw/airborne/arch/sim/jsbsim_ahrs.d \
./sw/airborne/arch/sim/jsbsim_gps.d \
./sw/airborne/arch/sim/jsbsim_hw.d \
./sw/airborne/arch/sim/jsbsim_ir.d \
./sw/airborne/arch/sim/jsbsim_transport.d \
./sw/airborne/arch/sim/led_hw.d \
./sw/airborne/arch/sim/max1167_hw.d \
./sw/airborne/arch/sim/mcu_arch.d \
./sw/airborne/arch/sim/micromag_hw.d \
./sw/airborne/arch/sim/sim_adc_generic.d \
./sw/airborne/arch/sim/sim_ahrs.d \
./sw/airborne/arch/sim/sim_ap.d \
./sw/airborne/arch/sim/sim_baro.d \
./sw/airborne/arch/sim/sim_gps.d \
./sw/airborne/arch/sim/sim_ir.d \
./sw/airborne/arch/sim/sim_uart.d \
./sw/airborne/arch/sim/sim_uart_hw.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/sim/%.o: ../sw/airborne/arch/sim/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


