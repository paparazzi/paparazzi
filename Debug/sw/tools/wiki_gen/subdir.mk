################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/tools/wiki_gen/wiki_gen.c 

OBJS += \
./sw/tools/wiki_gen/wiki_gen.o 

C_DEPS += \
./sw/tools/wiki_gen/wiki_gen.d 


# Each subdirectory must supply rules for building sources it contributes
sw/tools/wiki_gen/%.o: ../sw/tools/wiki_gen/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


