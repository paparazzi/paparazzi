################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/vehicle_interface/vi.c \
../sw/airborne/modules/vehicle_interface/vi_datalink.c \
../sw/airborne/modules/vehicle_interface/vi_overo_link.c \
../sw/airborne/modules/vehicle_interface/vi_test_signal.c 

OBJS += \
./sw/airborne/modules/vehicle_interface/vi.o \
./sw/airborne/modules/vehicle_interface/vi_datalink.o \
./sw/airborne/modules/vehicle_interface/vi_overo_link.o \
./sw/airborne/modules/vehicle_interface/vi_test_signal.o 

C_DEPS += \
./sw/airborne/modules/vehicle_interface/vi.d \
./sw/airborne/modules/vehicle_interface/vi_datalink.d \
./sw/airborne/modules/vehicle_interface/vi_overo_link.d \
./sw/airborne/modules/vehicle_interface/vi_test_signal.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/vehicle_interface/%.o: ../sw/airborne/modules/vehicle_interface/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


