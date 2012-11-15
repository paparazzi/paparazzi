################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ground_segment/misc/davis2ivy.o \
../sw/ground_segment/misc/kestrel2ivy.o 

C_SRCS += \
../sw/ground_segment/misc/davis2ivy.c \
../sw/ground_segment/misc/kestrel2ivy.c 

OBJS += \
./sw/ground_segment/misc/davis2ivy.o \
./sw/ground_segment/misc/kestrel2ivy.o 

C_DEPS += \
./sw/ground_segment/misc/davis2ivy.d \
./sw/ground_segment/misc/kestrel2ivy.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ground_segment/misc/%.o: ../sw/ground_segment/misc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


