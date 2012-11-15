################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/non_ap/led_flasher/blitzer.c 

OBJS += \
./sw/airborne/firmwares/non_ap/led_flasher/blitzer.o 

C_DEPS += \
./sw/airborne/firmwares/non_ap/led_flasher/blitzer.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/non_ap/led_flasher/%.o: ../sw/airborne/firmwares/non_ap/led_flasher/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


