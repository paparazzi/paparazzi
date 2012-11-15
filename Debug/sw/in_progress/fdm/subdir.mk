################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/in_progress/fdm/fdm_step.c \
../sw/in_progress/fdm/fms_steps_attitude.c \
../sw/in_progress/fdm/fms_steps_position.c 

OBJS += \
./sw/in_progress/fdm/fdm_step.o \
./sw/in_progress/fdm/fms_steps_attitude.o \
./sw/in_progress/fdm/fms_steps_position.o 

C_DEPS += \
./sw/in_progress/fdm/fdm_step.d \
./sw/in_progress/fdm/fms_steps_attitude.d \
./sw/in_progress/fdm/fms_steps_position.d 


# Each subdirectory must supply rules for building sources it contributes
sw/in_progress/fdm/%.o: ../sw/in_progress/fdm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


