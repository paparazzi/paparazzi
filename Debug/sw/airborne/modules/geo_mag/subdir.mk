################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/geo_mag/geo_mag.c 

OBJS += \
./sw/airborne/modules/geo_mag/geo_mag.o 

C_DEPS += \
./sw/airborne/modules/geo_mag/geo_mag.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/geo_mag/%.o: ../sw/airborne/modules/geo_mag/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


