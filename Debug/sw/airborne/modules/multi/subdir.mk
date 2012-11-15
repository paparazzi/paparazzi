################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/multi/formation.c \
../sw/airborne/modules/multi/potential.c \
../sw/airborne/modules/multi/tcas.c 

OBJS += \
./sw/airborne/modules/multi/formation.o \
./sw/airborne/modules/multi/potential.o \
./sw/airborne/modules/multi/tcas.o 

C_DEPS += \
./sw/airborne/modules/multi/formation.d \
./sw/airborne/modules/multi/potential.d \
./sw/airborne/modules/multi/tcas.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/multi/%.o: ../sw/airborne/modules/multi/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


