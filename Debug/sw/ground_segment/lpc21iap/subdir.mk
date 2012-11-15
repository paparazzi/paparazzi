################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ground_segment/lpc21iap/elf.c \
../sw/ground_segment/lpc21iap/lpc21iap.c \
../sw/ground_segment/lpc21iap/lpcusb.c 

OBJS += \
./sw/ground_segment/lpc21iap/elf.o \
./sw/ground_segment/lpc21iap/lpc21iap.o \
./sw/ground_segment/lpc21iap/lpcusb.o 

C_DEPS += \
./sw/ground_segment/lpc21iap/elf.d \
./sw/ground_segment/lpc21iap/lpc21iap.d \
./sw/ground_segment/lpc21iap/lpcusb.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ground_segment/lpc21iap/%.o: ../sw/ground_segment/lpc21iap/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


