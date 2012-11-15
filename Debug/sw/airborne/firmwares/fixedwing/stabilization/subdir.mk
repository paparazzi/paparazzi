################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/fixedwing/stabilization/stabilization_adaptive.c \
../sw/airborne/firmwares/fixedwing/stabilization/stabilization_attitude.c 

OBJS += \
./sw/airborne/firmwares/fixedwing/stabilization/stabilization_adaptive.o \
./sw/airborne/firmwares/fixedwing/stabilization/stabilization_attitude.o 

C_DEPS += \
./sw/airborne/firmwares/fixedwing/stabilization/stabilization_adaptive.d \
./sw/airborne/firmwares/fixedwing/stabilization/stabilization_attitude.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/fixedwing/stabilization/%.o: ../sw/airborne/firmwares/fixedwing/stabilization/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


