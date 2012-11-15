################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/subsystems/datalink/downlink.c \
../sw/airborne/subsystems/datalink/pprz_transport.c \
../sw/airborne/subsystems/datalink/w5100.c \
../sw/airborne/subsystems/datalink/xbee.c 

OBJS += \
./sw/airborne/subsystems/datalink/downlink.o \
./sw/airborne/subsystems/datalink/pprz_transport.o \
./sw/airborne/subsystems/datalink/w5100.o \
./sw/airborne/subsystems/datalink/xbee.o 

C_DEPS += \
./sw/airborne/subsystems/datalink/downlink.d \
./sw/airborne/subsystems/datalink/pprz_transport.d \
./sw/airborne/subsystems/datalink/w5100.d \
./sw/airborne/subsystems/datalink/xbee.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/subsystems/datalink/%.o: ../sw/airborne/subsystems/datalink/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


