################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/firmwares/tutorial/main_demo1.c \
../sw/airborne/firmwares/tutorial/main_demo2.c \
../sw/airborne/firmwares/tutorial/main_demo3.c \
../sw/airborne/firmwares/tutorial/main_demo4.c \
../sw/airborne/firmwares/tutorial/main_demo5.c \
../sw/airborne/firmwares/tutorial/main_demo6.c 

OBJS += \
./sw/airborne/firmwares/tutorial/main_demo1.o \
./sw/airborne/firmwares/tutorial/main_demo2.o \
./sw/airborne/firmwares/tutorial/main_demo3.o \
./sw/airborne/firmwares/tutorial/main_demo4.o \
./sw/airborne/firmwares/tutorial/main_demo5.o \
./sw/airborne/firmwares/tutorial/main_demo6.o 

C_DEPS += \
./sw/airborne/firmwares/tutorial/main_demo1.d \
./sw/airborne/firmwares/tutorial/main_demo2.d \
./sw/airborne/firmwares/tutorial/main_demo3.d \
./sw/airborne/firmwares/tutorial/main_demo4.d \
./sw/airborne/firmwares/tutorial/main_demo5.d \
./sw/airborne/firmwares/tutorial/main_demo6.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/firmwares/tutorial/%.o: ../sw/airborne/firmwares/tutorial/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


