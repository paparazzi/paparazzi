################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/benchmark/flight_benchmark.c \
../sw/airborne/modules/benchmark/i2c_abuse_test.c 

OBJS += \
./sw/airborne/modules/benchmark/flight_benchmark.o \
./sw/airborne/modules/benchmark/i2c_abuse_test.o 

C_DEPS += \
./sw/airborne/modules/benchmark/flight_benchmark.d \
./sw/airborne/modules/benchmark/i2c_abuse_test.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/benchmark/%.o: ../sw/airborne/modules/benchmark/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


