################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/ground_segment/tmtc/GSM/SMS_Ground_UDtest_final.c 

OBJS += \
./sw/ground_segment/tmtc/GSM/SMS_Ground_UDtest_final.o 

C_DEPS += \
./sw/ground_segment/tmtc/GSM/SMS_Ground_UDtest_final.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ground_segment/tmtc/GSM/%.o: ../sw/ground_segment/tmtc/GSM/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


