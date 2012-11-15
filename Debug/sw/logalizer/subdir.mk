################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/logalizer/export.o \
../sw/logalizer/gtk_export.o \
../sw/logalizer/log_file.o \
../sw/logalizer/plot.o 

C_SRCS += \
../sw/logalizer/ahrs2fg.c \
../sw/logalizer/ahrsview.c \
../sw/logalizer/ctrlstick.c \
../sw/logalizer/disp3d.c \
../sw/logalizer/ffjoystick.c \
../sw/logalizer/flight_gear.c \
../sw/logalizer/imuview.c \
../sw/logalizer/ivy_example.c \
../sw/logalizer/motor_bench.c \
../sw/logalizer/network.c \
../sw/logalizer/openlog2tlm.c \
../sw/logalizer/plot3dparse.c \
../sw/logalizer/plot_roll_loop.c \
../sw/logalizer/plotprofile.c \
../sw/logalizer/sliding_plot.c \
../sw/logalizer/test_2.c \
../sw/logalizer/tmclient.c \
../sw/logalizer/tmdata.c \
../sw/logalizer/tmserver.c \
../sw/logalizer/utils.c 

OBJS += \
./sw/logalizer/ahrs2fg.o \
./sw/logalizer/ahrsview.o \
./sw/logalizer/ctrlstick.o \
./sw/logalizer/disp3d.o \
./sw/logalizer/ffjoystick.o \
./sw/logalizer/flight_gear.o \
./sw/logalizer/imuview.o \
./sw/logalizer/ivy_example.o \
./sw/logalizer/motor_bench.o \
./sw/logalizer/network.o \
./sw/logalizer/openlog2tlm.o \
./sw/logalizer/plot3dparse.o \
./sw/logalizer/plot_roll_loop.o \
./sw/logalizer/plotprofile.o \
./sw/logalizer/sliding_plot.o \
./sw/logalizer/test_2.o \
./sw/logalizer/tmclient.o \
./sw/logalizer/tmdata.o \
./sw/logalizer/tmserver.o \
./sw/logalizer/utils.o 

C_DEPS += \
./sw/logalizer/ahrs2fg.d \
./sw/logalizer/ahrsview.d \
./sw/logalizer/ctrlstick.d \
./sw/logalizer/disp3d.d \
./sw/logalizer/ffjoystick.d \
./sw/logalizer/flight_gear.d \
./sw/logalizer/imuview.d \
./sw/logalizer/ivy_example.d \
./sw/logalizer/motor_bench.d \
./sw/logalizer/network.d \
./sw/logalizer/openlog2tlm.d \
./sw/logalizer/plot3dparse.d \
./sw/logalizer/plot_roll_loop.d \
./sw/logalizer/plotprofile.d \
./sw/logalizer/sliding_plot.d \
./sw/logalizer/test_2.d \
./sw/logalizer/tmclient.d \
./sw/logalizer/tmdata.d \
./sw/logalizer/tmserver.d \
./sw/logalizer/utils.d 


# Each subdirectory must supply rules for building sources it contributes
sw/logalizer/%.o: ../sw/logalizer/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


