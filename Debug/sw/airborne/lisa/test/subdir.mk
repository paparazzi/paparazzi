################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/lisa/test/hs_gyro.c \
../sw/airborne/lisa/test/lisa_test_actuators_mkk.c \
../sw/airborne/lisa/test/lisa_test_adxl345.c \
../sw/airborne/lisa/test/lisa_test_adxl345_dma.c \
../sw/airborne/lisa/test/lisa_test_aspirin.c \
../sw/airborne/lisa/test/lisa_test_hmc5843.c \
../sw/airborne/lisa/test/lisa_test_itg3200.c \
../sw/airborne/lisa/test/lisa_test_max1168.c \
../sw/airborne/lisa/test/lisa_test_ms2100.c \
../sw/airborne/lisa/test/lisa_test_sc18is600.c \
../sw/airborne/lisa/test/lisa_tunnel.c \
../sw/airborne/lisa/test/test_board.c \
../sw/airborne/lisa/test/test_bswap.c \
../sw/airborne/lisa/test/test_mc_asctec_v1_simple.c 

OBJS += \
./sw/airborne/lisa/test/hs_gyro.o \
./sw/airborne/lisa/test/lisa_test_actuators_mkk.o \
./sw/airborne/lisa/test/lisa_test_adxl345.o \
./sw/airborne/lisa/test/lisa_test_adxl345_dma.o \
./sw/airborne/lisa/test/lisa_test_aspirin.o \
./sw/airborne/lisa/test/lisa_test_hmc5843.o \
./sw/airborne/lisa/test/lisa_test_itg3200.o \
./sw/airborne/lisa/test/lisa_test_max1168.o \
./sw/airborne/lisa/test/lisa_test_ms2100.o \
./sw/airborne/lisa/test/lisa_test_sc18is600.o \
./sw/airborne/lisa/test/lisa_tunnel.o \
./sw/airborne/lisa/test/test_board.o \
./sw/airborne/lisa/test/test_bswap.o \
./sw/airborne/lisa/test/test_mc_asctec_v1_simple.o 

C_DEPS += \
./sw/airborne/lisa/test/hs_gyro.d \
./sw/airborne/lisa/test/lisa_test_actuators_mkk.d \
./sw/airborne/lisa/test/lisa_test_adxl345.d \
./sw/airborne/lisa/test/lisa_test_adxl345_dma.d \
./sw/airborne/lisa/test/lisa_test_aspirin.d \
./sw/airborne/lisa/test/lisa_test_hmc5843.d \
./sw/airborne/lisa/test/lisa_test_itg3200.d \
./sw/airborne/lisa/test/lisa_test_max1168.d \
./sw/airborne/lisa/test/lisa_test_ms2100.d \
./sw/airborne/lisa/test/lisa_test_sc18is600.d \
./sw/airborne/lisa/test/lisa_tunnel.d \
./sw/airborne/lisa/test/test_board.d \
./sw/airborne/lisa/test/test_bswap.d \
./sw/airborne/lisa/test/test_mc_asctec_v1_simple.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/lisa/test/%.o: ../sw/airborne/lisa/test/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


