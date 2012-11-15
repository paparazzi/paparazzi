################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/in_progress/ir_calibration/calibrator.c \
../sw/in_progress/ir_calibration/gui.c \
../sw/in_progress/ir_calibration/main.c 

OBJS += \
./sw/in_progress/ir_calibration/calibrator.o \
./sw/in_progress/ir_calibration/gui.o \
./sw/in_progress/ir_calibration/main.o 

C_DEPS += \
./sw/in_progress/ir_calibration/calibrator.d \
./sw/in_progress/ir_calibration/gui.d \
./sw/in_progress/ir_calibration/main.d 


# Each subdirectory must supply rules for building sources it contributes
sw/in_progress/ir_calibration/%.o: ../sw/in_progress/ir_calibration/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


