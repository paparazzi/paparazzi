################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/lpc21/test/ledswitch.c \
../sw/airborne/arch/lpc21/test/timer.c \
../sw/airborne/arch/lpc21/test/uart.c \
../sw/airborne/arch/lpc21/test/uart_tunnel.c 

S_UPPER_SRCS += \
../sw/airborne/arch/lpc21/test/crt0.S 

OBJS += \
./sw/airborne/arch/lpc21/test/crt0.o \
./sw/airborne/arch/lpc21/test/ledswitch.o \
./sw/airborne/arch/lpc21/test/timer.o \
./sw/airborne/arch/lpc21/test/uart.o \
./sw/airborne/arch/lpc21/test/uart_tunnel.o 

C_DEPS += \
./sw/airborne/arch/lpc21/test/ledswitch.d \
./sw/airborne/arch/lpc21/test/timer.d \
./sw/airborne/arch/lpc21/test/uart.d \
./sw/airborne/arch/lpc21/test/uart_tunnel.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/lpc21/test/%.o: ../sw/airborne/arch/lpc21/test/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GCC Assembler'
	as  -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

sw/airborne/arch/lpc21/test/%.o: ../sw/airborne/arch/lpc21/test/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


