################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/in_progress/satcom/email2udp.c \
../sw/in_progress/satcom/tcp2ivy.c \
../sw/in_progress/satcom/tcp2ivy_generic.c \
../sw/in_progress/satcom/udp2tcp.c 

OBJS += \
./sw/in_progress/satcom/email2udp.o \
./sw/in_progress/satcom/tcp2ivy.o \
./sw/in_progress/satcom/tcp2ivy_generic.o \
./sw/in_progress/satcom/udp2tcp.o 

C_DEPS += \
./sw/in_progress/satcom/email2udp.d \
./sw/in_progress/satcom/tcp2ivy.d \
./sw/in_progress/satcom/tcp2ivy_generic.d \
./sw/in_progress/satcom/udp2tcp.d 


# Each subdirectory must supply rules for building sources it contributes
sw/in_progress/satcom/%.o: ../sw/in_progress/satcom/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


