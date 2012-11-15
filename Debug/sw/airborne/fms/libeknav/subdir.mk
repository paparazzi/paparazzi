################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/fms/libeknav/estimate_attitude.c \
../sw/airborne/fms/libeknav/raw_log_to_ascii.c 

OBJS += \
./sw/airborne/fms/libeknav/estimate_attitude.o \
./sw/airborne/fms/libeknav/raw_log_to_ascii.o 

C_DEPS += \
./sw/airborne/fms/libeknav/estimate_attitude.d \
./sw/airborne/fms/libeknav/raw_log_to_ascii.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/fms/libeknav/%.o: ../sw/airborne/fms/libeknav/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


