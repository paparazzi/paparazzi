################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/subsystems/navigation/OSAMNav.c \
../sw/airborne/subsystems/navigation/bomb.c \
../sw/airborne/subsystems/navigation/border_line.c \
../sw/airborne/subsystems/navigation/common_flight_plan.c \
../sw/airborne/subsystems/navigation/common_nav.c \
../sw/airborne/subsystems/navigation/discsurvey.c \
../sw/airborne/subsystems/navigation/flightzone.c \
../sw/airborne/subsystems/navigation/gls.c \
../sw/airborne/subsystems/navigation/nav_cube.c \
../sw/airborne/subsystems/navigation/nav_line.c \
../sw/airborne/subsystems/navigation/nav_survey_rectangle.c \
../sw/airborne/subsystems/navigation/poly_survey_adv.c \
../sw/airborne/subsystems/navigation/snav.c \
../sw/airborne/subsystems/navigation/spiral.c \
../sw/airborne/subsystems/navigation/traffic_info.c 

OBJS += \
./sw/airborne/subsystems/navigation/OSAMNav.o \
./sw/airborne/subsystems/navigation/bomb.o \
./sw/airborne/subsystems/navigation/border_line.o \
./sw/airborne/subsystems/navigation/common_flight_plan.o \
./sw/airborne/subsystems/navigation/common_nav.o \
./sw/airborne/subsystems/navigation/discsurvey.o \
./sw/airborne/subsystems/navigation/flightzone.o \
./sw/airborne/subsystems/navigation/gls.o \
./sw/airborne/subsystems/navigation/nav_cube.o \
./sw/airborne/subsystems/navigation/nav_line.o \
./sw/airborne/subsystems/navigation/nav_survey_rectangle.o \
./sw/airborne/subsystems/navigation/poly_survey_adv.o \
./sw/airborne/subsystems/navigation/snav.o \
./sw/airborne/subsystems/navigation/spiral.o \
./sw/airborne/subsystems/navigation/traffic_info.o 

C_DEPS += \
./sw/airborne/subsystems/navigation/OSAMNav.d \
./sw/airborne/subsystems/navigation/bomb.d \
./sw/airborne/subsystems/navigation/border_line.d \
./sw/airborne/subsystems/navigation/common_flight_plan.d \
./sw/airborne/subsystems/navigation/common_nav.d \
./sw/airborne/subsystems/navigation/discsurvey.d \
./sw/airborne/subsystems/navigation/flightzone.d \
./sw/airborne/subsystems/navigation/gls.d \
./sw/airborne/subsystems/navigation/nav_cube.d \
./sw/airborne/subsystems/navigation/nav_line.d \
./sw/airborne/subsystems/navigation/nav_survey_rectangle.d \
./sw/airborne/subsystems/navigation/poly_survey_adv.d \
./sw/airborne/subsystems/navigation/snav.d \
./sw/airborne/subsystems/navigation/spiral.d \
./sw/airborne/subsystems/navigation/traffic_info.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/subsystems/navigation/%.o: ../sw/airborne/subsystems/navigation/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


