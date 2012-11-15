################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ext/luftboot/examples/blinky/blinky.c 

OBJS += \
./sw/ext/luftboot/examples/blinky/blinky.o 

C_DEPS += \
./sw/ext/luftboot/examples/blinky/blinky.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/luftboot/examples/blinky/%.o: ../sw/ext/luftboot/examples/blinky/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


