################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/cam_control/cam.c \
../sw/airborne/modules/cam_control/cam_roll.c \
../sw/airborne/modules/cam_control/cam_segment.c \
../sw/airborne/modules/cam_control/cam_track.c \
../sw/airborne/modules/cam_control/point.c \
../sw/airborne/modules/cam_control/rotorcraft_cam.c 

OBJS += \
./sw/airborne/modules/cam_control/cam.o \
./sw/airborne/modules/cam_control/cam_roll.o \
./sw/airborne/modules/cam_control/cam_segment.o \
./sw/airborne/modules/cam_control/cam_track.o \
./sw/airborne/modules/cam_control/point.o \
./sw/airborne/modules/cam_control/rotorcraft_cam.o 

C_DEPS += \
./sw/airborne/modules/cam_control/cam.d \
./sw/airborne/modules/cam_control/cam_roll.d \
./sw/airborne/modules/cam_control/cam_segment.d \
./sw/airborne/modules/cam_control/cam_track.d \
./sw/airborne/modules/cam_control/point.d \
./sw/airborne/modules/cam_control/rotorcraft_cam.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/cam_control/%.o: ../sw/airborne/modules/cam_control/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


