################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ground_segment/cockpit/actuators.c \
../sw/ground_segment/cockpit/ant_track.c \
../sw/ground_segment/cockpit/ant_track_pmm.c 

OBJS += \
./sw/ground_segment/cockpit/actuators.o \
./sw/ground_segment/cockpit/ant_track.o \
./sw/ground_segment/cockpit/ant_track_pmm.o 

C_DEPS += \
./sw/ground_segment/cockpit/actuators.d \
./sw/ground_segment/cockpit/ant_track.d \
./sw/ground_segment/cockpit/ant_track_pmm.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ground_segment/cockpit/%.o: ../sw/ground_segment/cockpit/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


