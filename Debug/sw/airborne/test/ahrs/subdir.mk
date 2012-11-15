################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/test/ahrs/ahrs_on_synth.c \
../sw/airborne/test/ahrs/run_ahrs_on_flight_log.c \
../sw/airborne/test/ahrs/run_ahrs_on_synth.c \
../sw/airborne/test/ahrs/run_ahrs_on_synth_ivy.c 

OBJS += \
./sw/airborne/test/ahrs/ahrs_on_synth.o \
./sw/airborne/test/ahrs/run_ahrs_on_flight_log.o \
./sw/airborne/test/ahrs/run_ahrs_on_synth.o \
./sw/airborne/test/ahrs/run_ahrs_on_synth_ivy.o 

C_DEPS += \
./sw/airborne/test/ahrs/ahrs_on_synth.d \
./sw/airborne/test/ahrs/run_ahrs_on_flight_log.d \
./sw/airborne/test/ahrs/run_ahrs_on_synth.d \
./sw/airborne/test/ahrs/run_ahrs_on_synth_ivy.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/test/ahrs/%.o: ../sw/airborne/test/ahrs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


