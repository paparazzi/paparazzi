################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/modules/enose/anemotaxis.c \
../sw/airborne/modules/enose/chemo_detect.c \
../sw/airborne/modules/enose/chemotaxis.c \
../sw/airborne/modules/enose/enose.c 

OBJS += \
./sw/airborne/modules/enose/anemotaxis.o \
./sw/airborne/modules/enose/chemo_detect.o \
./sw/airborne/modules/enose/chemotaxis.o \
./sw/airborne/modules/enose/enose.o 

C_DEPS += \
./sw/airborne/modules/enose/anemotaxis.d \
./sw/airborne/modules/enose/chemo_detect.d \
./sw/airborne/modules/enose/chemotaxis.d \
./sw/airborne/modules/enose/enose.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/modules/enose/%.o: ../sw/airborne/modules/enose/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


