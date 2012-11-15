################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/airborne_ant_track/airborne_ant_track.c 

OBJS += \
./sw/airborne/modules/airborne_ant_track/airborne_ant_track.o 

C_DEPS += \
./sw/airborne/modules/airborne_ant_track/airborne_ant_track.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/airborne_ant_track/%.o: ../sw/airborne/modules/airborne_ant_track/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


