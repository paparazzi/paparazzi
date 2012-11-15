################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/cartography/cartography.c \
../sw/airborne/modules/cartography/photogrammetry_calculator.c 

OBJS += \
./sw/airborne/modules/cartography/cartography.o \
./sw/airborne/modules/cartography/photogrammetry_calculator.o 

C_DEPS += \
./sw/airborne/modules/cartography/cartography.d \
./sw/airborne/modules/cartography/photogrammetry_calculator.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/cartography/%.o: ../sw/airborne/modules/cartography/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


