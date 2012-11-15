################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/digital_cam/atmega_i2c_cam_ctrl.c \
../sw/airborne/modules/digital_cam/dc.c \
../sw/airborne/modules/digital_cam/led_cam_ctrl.c \
../sw/airborne/modules/digital_cam/servo_cam_ctrl.c \
../sw/airborne/modules/digital_cam/sim_i2c_cam_ctrl.c 

OBJS += \
./sw/airborne/modules/digital_cam/atmega_i2c_cam_ctrl.o \
./sw/airborne/modules/digital_cam/dc.o \
./sw/airborne/modules/digital_cam/led_cam_ctrl.o \
./sw/airborne/modules/digital_cam/servo_cam_ctrl.o \
./sw/airborne/modules/digital_cam/sim_i2c_cam_ctrl.o 

C_DEPS += \
./sw/airborne/modules/digital_cam/atmega_i2c_cam_ctrl.d \
./sw/airborne/modules/digital_cam/dc.d \
./sw/airborne/modules/digital_cam/led_cam_ctrl.d \
./sw/airborne/modules/digital_cam/servo_cam_ctrl.d \
./sw/airborne/modules/digital_cam/sim_i2c_cam_ctrl.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/digital_cam/%.o: ../sw/airborne/modules/digital_cam/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


