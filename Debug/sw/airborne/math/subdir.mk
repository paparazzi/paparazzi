################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/math/pprz_geodetic_double.c \
../sw/airborne/math/pprz_geodetic_float.c \
../sw/airborne/math/pprz_geodetic_int.c \
../sw/airborne/math/pprz_geodetic_wmm2010.c \
../sw/airborne/math/pprz_orientation_conversion.c \
../sw/airborne/math/pprz_trig_int.c 

OBJS += \
./sw/airborne/math/pprz_geodetic_double.o \
./sw/airborne/math/pprz_geodetic_float.o \
./sw/airborne/math/pprz_geodetic_int.o \
./sw/airborne/math/pprz_geodetic_wmm2010.o \
./sw/airborne/math/pprz_orientation_conversion.o \
./sw/airborne/math/pprz_trig_int.o 

C_DEPS += \
./sw/airborne/math/pprz_geodetic_double.d \
./sw/airborne/math/pprz_geodetic_float.d \
./sw/airborne/math/pprz_geodetic_int.d \
./sw/airborne/math/pprz_geodetic_wmm2010.d \
./sw/airborne/math/pprz_orientation_conversion.d \
./sw/airborne/math/pprz_trig_int.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/math/%.o: ../sw/airborne/math/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


