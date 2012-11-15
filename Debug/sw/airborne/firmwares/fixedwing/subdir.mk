################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/fixedwing/ap_downlink.c \
../sw/airborne/firmwares/fixedwing/autopilot.c \
../sw/airborne/firmwares/fixedwing/datalink.c \
../sw/airborne/firmwares/fixedwing/fbw_downlink.c \
../sw/airborne/firmwares/fixedwing/main.c \
../sw/airborne/firmwares/fixedwing/main_ap.c \
../sw/airborne/firmwares/fixedwing/main_fbw.c 

OBJS += \
./sw/airborne/firmwares/fixedwing/ap_downlink.o \
./sw/airborne/firmwares/fixedwing/autopilot.o \
./sw/airborne/firmwares/fixedwing/datalink.o \
./sw/airborne/firmwares/fixedwing/fbw_downlink.o \
./sw/airborne/firmwares/fixedwing/main.o \
./sw/airborne/firmwares/fixedwing/main_ap.o \
./sw/airborne/firmwares/fixedwing/main_fbw.o 

C_DEPS += \
./sw/airborne/firmwares/fixedwing/ap_downlink.d \
./sw/airborne/firmwares/fixedwing/autopilot.d \
./sw/airborne/firmwares/fixedwing/datalink.d \
./sw/airborne/firmwares/fixedwing/fbw_downlink.d \
./sw/airborne/firmwares/fixedwing/main.d \
./sw/airborne/firmwares/fixedwing/main_ap.d \
./sw/airborne/firmwares/fixedwing/main_fbw.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/fixedwing/%.o: ../sw/airborne/firmwares/fixedwing/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


