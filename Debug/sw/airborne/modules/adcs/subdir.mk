################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/adcs/adc_generic.c \
../sw/airborne/modules/adcs/max11040.c 

OBJS += \
./sw/airborne/modules/adcs/adc_generic.o \
./sw/airborne/modules/adcs/max11040.o 

C_DEPS += \
./sw/airborne/modules/adcs/adc_generic.d \
./sw/airborne/modules/adcs/max11040.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/adcs/%.o: ../sw/airborne/modules/adcs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


