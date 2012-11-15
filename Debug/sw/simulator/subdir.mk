################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/simulator/fg.o 

C_SRCS += \
../sw/simulator/fg.c \
../sw/simulator/sim_ac_booz.c \
../sw/simulator/sim_ac_flightgear.c \
../sw/simulator/sim_ac_fw.c \
../sw/simulator/sim_ac_jsbsim.c 

OBJS += \
./sw/simulator/fg.o \
./sw/simulator/sim_ac_booz.o \
./sw/simulator/sim_ac_flightgear.o \
./sw/simulator/sim_ac_fw.o \
./sw/simulator/sim_ac_jsbsim.o 

C_DEPS += \
./sw/simulator/fg.d \
./sw/simulator/sim_ac_booz.d \
./sw/simulator/sim_ac_flightgear.d \
./sw/simulator/sim_ac_fw.d \
./sw/simulator/sim_ac_jsbsim.d 


# Each subdirectory must supply rules for building sources it contributes
sw/simulator/%.o: ../sw/simulator/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


