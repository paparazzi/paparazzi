################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ext/libopencm3/lib/lpc43xx/assert.o \
../sw/ext/libopencm3/lib/lpc43xx/gpio.o \
../sw/ext/libopencm3/lib/lpc43xx/i2c.o \
../sw/ext/libopencm3/lib/lpc43xx/nvic.o \
../sw/ext/libopencm3/lib/lpc43xx/scb.o \
../sw/ext/libopencm3/lib/lpc43xx/scu.o \
../sw/ext/libopencm3/lib/lpc43xx/ssp.o \
../sw/ext/libopencm3/lib/lpc43xx/systick.o \
../sw/ext/libopencm3/lib/lpc43xx/vector.o 

C_SRCS += \
../sw/ext/libopencm3/lib/lpc43xx/gpio.c \
../sw/ext/libopencm3/lib/lpc43xx/i2c.c \
../sw/ext/libopencm3/lib/lpc43xx/scu.c \
../sw/ext/libopencm3/lib/lpc43xx/ssp.c \
../sw/ext/libopencm3/lib/lpc43xx/vector_chipset.c \
../sw/ext/libopencm3/lib/lpc43xx/vector_nvic.c 

OBJS += \
./sw/ext/libopencm3/lib/lpc43xx/gpio.o \
./sw/ext/libopencm3/lib/lpc43xx/i2c.o \
./sw/ext/libopencm3/lib/lpc43xx/scu.o \
./sw/ext/libopencm3/lib/lpc43xx/ssp.o \
./sw/ext/libopencm3/lib/lpc43xx/vector_chipset.o \
./sw/ext/libopencm3/lib/lpc43xx/vector_nvic.o 

C_DEPS += \
./sw/ext/libopencm3/lib/lpc43xx/gpio.d \
./sw/ext/libopencm3/lib/lpc43xx/i2c.d \
./sw/ext/libopencm3/lib/lpc43xx/scu.d \
./sw/ext/libopencm3/lib/lpc43xx/ssp.d \
./sw/ext/libopencm3/lib/lpc43xx/vector_chipset.d \
./sw/ext/libopencm3/lib/lpc43xx/vector_nvic.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ext/libopencm3/lib/lpc43xx/%.o: ../sw/ext/libopencm3/lib/lpc43xx/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


