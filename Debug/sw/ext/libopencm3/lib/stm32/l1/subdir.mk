################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ext/libopencm3/lib/stm32/l1/assert.o \
../sw/ext/libopencm3/lib/stm32/l1/crc.o \
../sw/ext/libopencm3/lib/stm32/l1/desig.o \
../sw/ext/libopencm3/lib/stm32/l1/exti2.o \
../sw/ext/libopencm3/lib/stm32/l1/gpio2.o \
../sw/ext/libopencm3/lib/stm32/l1/nvic.o \
../sw/ext/libopencm3/lib/stm32/l1/rcc.o \
../sw/ext/libopencm3/lib/stm32/l1/scb.o \
../sw/ext/libopencm3/lib/stm32/l1/systick.o \
../sw/ext/libopencm3/lib/stm32/l1/usart.o \
../sw/ext/libopencm3/lib/stm32/l1/vector.o 

C_SRCS += \
../sw/ext/libopencm3/lib/stm32/l1/rcc.c \
../sw/ext/libopencm3/lib/stm32/l1/vector_nvic.c 

OBJS += \
./sw/ext/libopencm3/lib/stm32/l1/rcc.o \
./sw/ext/libopencm3/lib/stm32/l1/vector_nvic.o 

C_DEPS += \
./sw/ext/libopencm3/lib/stm32/l1/rcc.d \
./sw/ext/libopencm3/lib/stm32/l1/vector_nvic.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/stm32/l1/%.o: ../sw/ext/libopencm3/lib/stm32/l1/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


