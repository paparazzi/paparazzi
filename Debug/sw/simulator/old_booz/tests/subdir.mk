################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/simulator/old_booz/tests/test_fdm.c \
../sw/simulator/old_booz/tests/test_sensors.c 

OBJS += \
./sw/simulator/old_booz/tests/test_fdm.o \
./sw/simulator/old_booz/tests/test_sensors.o 

C_DEPS += \
./sw/simulator/old_booz/tests/test_fdm.d \
./sw/simulator/old_booz/tests/test_sensors.d 


# Each subdirectory must supply rules for building sources it contributes
sw/simulator/old_booz/tests/%.o: ../sw/simulator/old_booz/tests/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


