################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/sd_card/main.c \
../sw/airborne/sd_card/sd_card.c 

OBJS += \
./sw/airborne/sd_card/main.o \
./sw/airborne/sd_card/sd_card.o 

C_DEPS += \
./sw/airborne/sd_card/main.d \
./sw/airborne/sd_card/sd_card.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/sd_card/%.o: ../sw/airborne/sd_card/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


