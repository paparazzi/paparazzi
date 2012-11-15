################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/subsystems/ins/hf_float.c \
../sw/airborne/subsystems/ins/ins_alt_float.c \
../sw/airborne/subsystems/ins/ins_gps_passthrough.c \
../sw/airborne/subsystems/ins/ins_int.c \
../sw/airborne/subsystems/ins/ins_int_extended.c \
../sw/airborne/subsystems/ins/vf_extended_float.c \
../sw/airborne/subsystems/ins/vf_float.c \
../sw/airborne/subsystems/ins/vf_int.c 

OBJS += \
./sw/airborne/subsystems/ins/hf_float.o \
./sw/airborne/subsystems/ins/ins_alt_float.o \
./sw/airborne/subsystems/ins/ins_gps_passthrough.o \
./sw/airborne/subsystems/ins/ins_int.o \
./sw/airborne/subsystems/ins/ins_int_extended.o \
./sw/airborne/subsystems/ins/vf_extended_float.o \
./sw/airborne/subsystems/ins/vf_float.o \
./sw/airborne/subsystems/ins/vf_int.o 

C_DEPS += \
./sw/airborne/subsystems/ins/hf_float.d \
./sw/airborne/subsystems/ins/ins_alt_float.d \
./sw/airborne/subsystems/ins/ins_gps_passthrough.d \
./sw/airborne/subsystems/ins/ins_int.d \
./sw/airborne/subsystems/ins/ins_int_extended.d \
./sw/airborne/subsystems/ins/vf_extended_float.d \
./sw/airborne/subsystems/ins/vf_float.d \
./sw/airborne/subsystems/ins/vf_int.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/subsystems/ins/%.o: ../sw/airborne/subsystems/ins/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


