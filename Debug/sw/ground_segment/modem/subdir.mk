################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ground_segment/modem/adc.c \
../sw/ground_segment/modem/main.c \
../sw/ground_segment/modem/soft_uart.c \
../sw/ground_segment/modem/uart.c 

OBJS += \
./sw/ground_segment/modem/adc.o \
./sw/ground_segment/modem/main.o \
./sw/ground_segment/modem/soft_uart.o \
./sw/ground_segment/modem/uart.o 

C_DEPS += \
./sw/ground_segment/modem/adc.d \
./sw/ground_segment/modem/main.d \
./sw/ground_segment/modem/soft_uart.d \
./sw/ground_segment/modem/uart.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ground_segment/modem/%.o: ../sw/ground_segment/modem/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


