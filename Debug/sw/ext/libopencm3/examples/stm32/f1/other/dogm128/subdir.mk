################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ext/libopencm3/examples/stm32/f1/other/dogm128/dogm128.c \
../sw/ext/libopencm3/examples/stm32/f1/other/dogm128/main.c 

OBJS += \
./sw/ext/libopencm3/examples/stm32/f1/other/dogm128/dogm128.o \
./sw/ext/libopencm3/examples/stm32/f1/other/dogm128/main.o 

C_DEPS += \
./sw/ext/libopencm3/examples/stm32/f1/other/dogm128/dogm128.d \
./sw/ext/libopencm3/examples/stm32/f1/other/dogm128/main.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/examples/stm32/f1/other/dogm128/%.o: ../sw/ext/libopencm3/examples/stm32/f1/other/dogm128/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


