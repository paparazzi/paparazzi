################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/in_progress/rctx/main_rctx.c 

OBJS += \
./sw/in_progress/rctx/main_rctx.o 

C_DEPS += \
./sw/in_progress/rctx/main_rctx.d 


# Each subdirectory must supply rules for building sources it contributes
sw/in_progress/rctx/%.o: ../sw/in_progress/rctx/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


