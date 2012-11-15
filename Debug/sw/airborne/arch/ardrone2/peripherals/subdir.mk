################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/ardrone2/peripherals/hmc5843_arch.c \
../sw/airborne/arch/ardrone2/peripherals/max1168_arch.c \
../sw/airborne/arch/ardrone2/peripherals/ms2100_arch.c \
../sw/airborne/arch/ardrone2/peripherals/sc18is600_arch.c 

OBJS += \
./sw/airborne/arch/ardrone2/peripherals/hmc5843_arch.o \
./sw/airborne/arch/ardrone2/peripherals/max1168_arch.o \
./sw/airborne/arch/ardrone2/peripherals/ms2100_arch.o \
./sw/airborne/arch/ardrone2/peripherals/sc18is600_arch.o 

C_DEPS += \
./sw/airborne/arch/ardrone2/peripherals/hmc5843_arch.d \
./sw/airborne/arch/ardrone2/peripherals/max1168_arch.d \
./sw/airborne/arch/ardrone2/peripherals/ms2100_arch.d \
./sw/airborne/arch/ardrone2/peripherals/sc18is600_arch.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/ardrone2/peripherals/%.o: ../sw/airborne/arch/ardrone2/peripherals/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


