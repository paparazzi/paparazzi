################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/rotorcraft/guidance/guidance_h.c \
../sw/airborne/firmwares/rotorcraft/guidance/guidance_v.c 

OBJS += \
./sw/airborne/firmwares/rotorcraft/guidance/guidance_h.o \
./sw/airborne/firmwares/rotorcraft/guidance/guidance_v.o 

C_DEPS += \
./sw/airborne/firmwares/rotorcraft/guidance/guidance_h.d \
./sw/airborne/firmwares/rotorcraft/guidance/guidance_v.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/rotorcraft/guidance/%.o: ../sw/airborne/firmwares/rotorcraft/guidance/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


