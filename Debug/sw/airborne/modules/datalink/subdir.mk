################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/datalink/extra_pprz_dl.c \
../sw/airborne/modules/datalink/xtend_rssi.c 

OBJS += \
./sw/airborne/modules/datalink/extra_pprz_dl.o \
./sw/airborne/modules/datalink/xtend_rssi.o 

C_DEPS += \
./sw/airborne/modules/datalink/extra_pprz_dl.d \
./sw/airborne/modules/datalink/xtend_rssi.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/datalink/%.o: ../sw/airborne/modules/datalink/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


