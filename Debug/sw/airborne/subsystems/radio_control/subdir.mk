################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/subsystems/radio_control/dummy.c \
../sw/airborne/subsystems/radio_control/joby.c \
../sw/airborne/subsystems/radio_control/ppm.c \
../sw/airborne/subsystems/radio_control/rc_datalink.c \
../sw/airborne/subsystems/radio_control/spektrum.c 

OBJS += \
./sw/airborne/subsystems/radio_control/dummy.o \
./sw/airborne/subsystems/radio_control/joby.o \
./sw/airborne/subsystems/radio_control/ppm.o \
./sw/airborne/subsystems/radio_control/rc_datalink.o \
./sw/airborne/subsystems/radio_control/spektrum.o 

C_DEPS += \
./sw/airborne/subsystems/radio_control/dummy.d \
./sw/airborne/subsystems/radio_control/joby.d \
./sw/airborne/subsystems/radio_control/ppm.d \
./sw/airborne/subsystems/radio_control/rc_datalink.d \
./sw/airborne/subsystems/radio_control/spektrum.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/subsystems/radio_control/%.o: ../sw/airborne/subsystems/radio_control/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


