################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/peripherals/ads1114.c \
../sw/airborne/peripherals/adxl345.i2c.c \
../sw/airborne/peripherals/ami601.c \
../sw/airborne/peripherals/hmc5843.c \
../sw/airborne/peripherals/hmc58xx.c \
../sw/airborne/peripherals/itg3200.c \
../sw/airborne/peripherals/max1168.c \
../sw/airborne/peripherals/mcp355x.c \
../sw/airborne/peripherals/ms2100.c \
../sw/airborne/peripherals/sc18i600.c 

OBJS += \
./sw/airborne/peripherals/ads1114.o \
./sw/airborne/peripherals/adxl345.i2c.o \
./sw/airborne/peripherals/ami601.o \
./sw/airborne/peripherals/hmc5843.o \
./sw/airborne/peripherals/hmc58xx.o \
./sw/airborne/peripherals/itg3200.o \
./sw/airborne/peripherals/max1168.o \
./sw/airborne/peripherals/mcp355x.o \
./sw/airborne/peripherals/ms2100.o \
./sw/airborne/peripherals/sc18i600.o 

C_DEPS += \
./sw/airborne/peripherals/ads1114.d \
./sw/airborne/peripherals/adxl345.i2c.d \
./sw/airborne/peripherals/ami601.d \
./sw/airborne/peripherals/hmc5843.d \
./sw/airborne/peripherals/hmc58xx.d \
./sw/airborne/peripherals/itg3200.d \
./sw/airborne/peripherals/max1168.d \
./sw/airborne/peripherals/mcp355x.d \
./sw/airborne/peripherals/ms2100.d \
./sw/airborne/peripherals/sc18i600.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/peripherals/%.o: ../sw/airborne/peripherals/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


