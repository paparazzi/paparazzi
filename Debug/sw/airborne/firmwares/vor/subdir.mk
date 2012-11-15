################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/vor/i86_vor_test_filters.c \
../sw/airborne/firmwares/vor/i86_vor_test_float_demod.c \
../sw/airborne/firmwares/vor/i86_vor_test_int_demod.c \
../sw/airborne/firmwares/vor/i86_vor_test_int_demod_decim.c \
../sw/airborne/firmwares/vor/lpc_vor_convertions.c \
../sw/airborne/firmwares/vor/lpc_vor_main.c \
../sw/airborne/firmwares/vor/vor_float_demod.c \
../sw/airborne/firmwares/vor/vor_int_demod.c \
../sw/airborne/firmwares/vor/vor_int_demod_decim.c 

OBJS += \
./sw/airborne/firmwares/vor/i86_vor_test_filters.o \
./sw/airborne/firmwares/vor/i86_vor_test_float_demod.o \
./sw/airborne/firmwares/vor/i86_vor_test_int_demod.o \
./sw/airborne/firmwares/vor/i86_vor_test_int_demod_decim.o \
./sw/airborne/firmwares/vor/lpc_vor_convertions.o \
./sw/airborne/firmwares/vor/lpc_vor_main.o \
./sw/airborne/firmwares/vor/vor_float_demod.o \
./sw/airborne/firmwares/vor/vor_int_demod.o \
./sw/airborne/firmwares/vor/vor_int_demod_decim.o 

C_DEPS += \
./sw/airborne/firmwares/vor/i86_vor_test_filters.d \
./sw/airborne/firmwares/vor/i86_vor_test_float_demod.d \
./sw/airborne/firmwares/vor/i86_vor_test_int_demod.d \
./sw/airborne/firmwares/vor/i86_vor_test_int_demod_decim.d \
./sw/airborne/firmwares/vor/lpc_vor_convertions.d \
./sw/airborne/firmwares/vor/lpc_vor_main.d \
./sw/airborne/firmwares/vor/vor_float_demod.d \
./sw/airborne/firmwares/vor/vor_int_demod.d \
./sw/airborne/firmwares/vor/vor_int_demod_decim.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/vor/%.o: ../sw/airborne/firmwares/vor/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


