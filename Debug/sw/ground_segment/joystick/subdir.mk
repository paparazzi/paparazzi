################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ground_segment/joystick/ml_sdl_stick.c \
../sw/ground_segment/joystick/ml_usb_stick.c \
../sw/ground_segment/joystick/sdl_stick.c \
../sw/ground_segment/joystick/test_sdl_stick.c \
../sw/ground_segment/joystick/test_stick.c \
../sw/ground_segment/joystick/usb_stick.c 

OBJS += \
./sw/ground_segment/joystick/ml_sdl_stick.o \
./sw/ground_segment/joystick/ml_usb_stick.o \
./sw/ground_segment/joystick/sdl_stick.o \
./sw/ground_segment/joystick/test_sdl_stick.o \
./sw/ground_segment/joystick/test_stick.o \
./sw/ground_segment/joystick/usb_stick.o 

C_DEPS += \
./sw/ground_segment/joystick/ml_sdl_stick.d \
./sw/ground_segment/joystick/ml_usb_stick.d \
./sw/ground_segment/joystick/sdl_stick.d \
./sw/ground_segment/joystick/test_sdl_stick.d \
./sw/ground_segment/joystick/test_stick.d \
./sw/ground_segment/joystick/usb_stick.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ground_segment/joystick/%.o: ../sw/ground_segment/joystick/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


