################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/fixedwing/guidance/energy_ctrl.c \
../sw/airborne/firmwares/fixedwing/guidance/guidance_v.c \
../sw/airborne/firmwares/fixedwing/guidance/guidance_v_n.c 

OBJS += \
./sw/airborne/firmwares/fixedwing/guidance/energy_ctrl.o \
./sw/airborne/firmwares/fixedwing/guidance/guidance_v.o \
./sw/airborne/firmwares/fixedwing/guidance/guidance_v_n.o 

C_DEPS += \
./sw/airborne/firmwares/fixedwing/guidance/energy_ctrl.d \
./sw/airborne/firmwares/fixedwing/guidance/guidance_v.d \
./sw/airborne/firmwares/fixedwing/guidance/guidance_v_n.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/fixedwing/guidance/%.o: ../sw/airborne/firmwares/fixedwing/guidance/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


