################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ext/luftboot/src/luftboot.o 

C_SRCS += \
../sw/ext/luftboot/src/luftboot.c 

OBJS += \
./sw/ext/luftboot/src/luftboot.o 

C_DEPS += \
./sw/ext/luftboot/src/luftboot.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/luftboot/src/%.o: ../sw/ext/luftboot/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


