################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/booz/test/booz2_test_buss_bldc_hexa.c \
../sw/airborne/booz/test/booz2_test_crista.c \
../sw/airborne/booz/test/booz2_test_gps.c \
../sw/airborne/booz/test/booz2_test_max1168.c \
../sw/airborne/booz/test/booz2_test_micromag.c \
../sw/airborne/booz/test/booz2_test_usb.c \
../sw/airborne/booz/test/test_mlkf.c \
../sw/airborne/booz/test/test_scaling.c \
../sw/airborne/booz/test/test_vg_adpt.c \
../sw/airborne/booz/test/test_vg_ref.c 

OBJS += \
./sw/airborne/booz/test/booz2_test_buss_bldc_hexa.o \
./sw/airborne/booz/test/booz2_test_crista.o \
./sw/airborne/booz/test/booz2_test_gps.o \
./sw/airborne/booz/test/booz2_test_max1168.o \
./sw/airborne/booz/test/booz2_test_micromag.o \
./sw/airborne/booz/test/booz2_test_usb.o \
./sw/airborne/booz/test/test_mlkf.o \
./sw/airborne/booz/test/test_scaling.o \
./sw/airborne/booz/test/test_vg_adpt.o \
./sw/airborne/booz/test/test_vg_ref.o 

C_DEPS += \
./sw/airborne/booz/test/booz2_test_buss_bldc_hexa.d \
./sw/airborne/booz/test/booz2_test_crista.d \
./sw/airborne/booz/test/booz2_test_gps.d \
./sw/airborne/booz/test/booz2_test_max1168.d \
./sw/airborne/booz/test/booz2_test_micromag.d \
./sw/airborne/booz/test/booz2_test_usb.d \
./sw/airborne/booz/test/test_mlkf.d \
./sw/airborne/booz/test/test_scaling.d \
./sw/airborne/booz/test/test_vg_adpt.d \
./sw/airborne/booz/test/test_vg_ref.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/booz/test/%.o: ../sw/airborne/booz/test/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


