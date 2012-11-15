################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../conf/gps/ublox_conf.c 

OBJS += \
./conf/gps/ublox_conf.o 

C_DEPS += \
./conf/gps/ublox_conf.d 


# Each subdirectory must supply rules for building sources it contributes
conf/gps/%.o: ../conf/gps/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


