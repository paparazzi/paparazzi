################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/in_progress/river_tracking/src/ivy_example.c \
../sw/in_progress/river_tracking/src/nextwp.c \
../sw/in_progress/river_tracking/src/river_track.c 

OBJS += \
./sw/in_progress/river_tracking/src/ivy_example.o \
./sw/in_progress/river_tracking/src/nextwp.o \
./sw/in_progress/river_tracking/src/river_track.o 

C_DEPS += \
./sw/in_progress/river_tracking/src/ivy_example.d \
./sw/in_progress/river_tracking/src/nextwp.d \
./sw/in_progress/river_tracking/src/river_track.d 


# Each subdirectory must supply rules for building sources it contributes
sw/in_progress/river_tracking/src/%.o: ../sw/in_progress/river_tracking/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


