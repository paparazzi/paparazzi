################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ground_segment/tmtc/c_ivy_client_example_1.c \
../sw/ground_segment/tmtc/c_ivy_client_example_2.c \
../sw/ground_segment/tmtc/c_ivy_client_example_3.c \
../sw/ground_segment/tmtc/gpsd2ivy.c \
../sw/ground_segment/tmtc/ivy_serial_bridge.c 

OBJS += \
./sw/ground_segment/tmtc/c_ivy_client_example_1.o \
./sw/ground_segment/tmtc/c_ivy_client_example_2.o \
./sw/ground_segment/tmtc/c_ivy_client_example_3.o \
./sw/ground_segment/tmtc/gpsd2ivy.o \
./sw/ground_segment/tmtc/ivy_serial_bridge.o 

C_DEPS += \
./sw/ground_segment/tmtc/c_ivy_client_example_1.d \
./sw/ground_segment/tmtc/c_ivy_client_example_2.d \
./sw/ground_segment/tmtc/c_ivy_client_example_3.d \
./sw/ground_segment/tmtc/gpsd2ivy.d \
./sw/ground_segment/tmtc/ivy_serial_bridge.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ground_segment/tmtc/%.o: ../sw/ground_segment/tmtc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


