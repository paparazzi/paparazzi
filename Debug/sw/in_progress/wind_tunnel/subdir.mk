################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/in_progress/wind_tunnel/main.c \
../sw/in_progress/wind_tunnel/serial_port.c 

OBJS += \
./sw/in_progress/wind_tunnel/main.o \
./sw/in_progress/wind_tunnel/serial_port.o 

C_DEPS += \
./sw/in_progress/wind_tunnel/main.d \
./sw/in_progress/wind_tunnel/serial_port.d 


# Each subdirectory must supply rules for building sources it contributes
sw/in_progress/wind_tunnel/%.o: ../sw/in_progress/wind_tunnel/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


