################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../sw/ground_segment/multimon/costabi.o \
../sw/ground_segment/multimon/demod_afsk12.o \
../sw/ground_segment/multimon/demod_afsk48p.o \
../sw/ground_segment/multimon/demod_display.o \
../sw/ground_segment/multimon/demodml.o \
../sw/ground_segment/multimon/gen_hdlc.o \
../sw/ground_segment/multimon/hdlc.o \
../sw/ground_segment/multimon/mkcostab.o \
../sw/ground_segment/multimon/ml_hdlc.o \
../sw/ground_segment/multimon/pprz.o \
../sw/ground_segment/multimon/pprzlib.o \
../sw/ground_segment/multimon/unixinput.o \
../sw/ground_segment/multimon/xdisplay.o 

C_SRCS += \
../sw/ground_segment/multimon/costabf.c \
../sw/ground_segment/multimon/costabi.c \
../sw/ground_segment/multimon/demod_afsk12.c \
../sw/ground_segment/multimon/demod_afsk48p.c \
../sw/ground_segment/multimon/demod_display.c \
../sw/ground_segment/multimon/demodml.c \
../sw/ground_segment/multimon/gen_hdlc.c \
../sw/ground_segment/multimon/hdlc.c \
../sw/ground_segment/multimon/mkcostab.c \
../sw/ground_segment/multimon/ml_hdlc.c \
../sw/ground_segment/multimon/pprz.c \
../sw/ground_segment/multimon/pprzlib.c \
../sw/ground_segment/multimon/unixinput.c \
../sw/ground_segment/multimon/xdisplay.c 

OBJS += \
./sw/ground_segment/multimon/costabf.o \
./sw/ground_segment/multimon/costabi.o \
./sw/ground_segment/multimon/demod_afsk12.o \
./sw/ground_segment/multimon/demod_afsk48p.o \
./sw/ground_segment/multimon/demod_display.o \
./sw/ground_segment/multimon/demodml.o \
./sw/ground_segment/multimon/gen_hdlc.o \
./sw/ground_segment/multimon/hdlc.o \
./sw/ground_segment/multimon/mkcostab.o \
./sw/ground_segment/multimon/ml_hdlc.o \
./sw/ground_segment/multimon/pprz.o \
./sw/ground_segment/multimon/pprzlib.o \
./sw/ground_segment/multimon/unixinput.o \
./sw/ground_segment/multimon/xdisplay.o 

C_DEPS += \
./sw/ground_segment/multimon/costabf.d \
./sw/ground_segment/multimon/costabi.d \
./sw/ground_segment/multimon/demod_afsk12.d \
./sw/ground_segment/multimon/demod_afsk48p.d \
./sw/ground_segment/multimon/demod_display.d \
./sw/ground_segment/multimon/demodml.d \
./sw/ground_segment/multimon/gen_hdlc.d \
./sw/ground_segment/multimon/hdlc.d \
./sw/ground_segment/multimon/mkcostab.d \
./sw/ground_segment/multimon/ml_hdlc.d \
./sw/ground_segment/multimon/pprz.d \
./sw/ground_segment/multimon/pprzlib.d \
./sw/ground_segment/multimon/unixinput.d \
./sw/ground_segment/multimon/xdisplay.d 


# Each subdirectory must supply rules for building sources it contributes
sw/ground_segment/multimon/%.o: ../sw/ground_segment/multimon/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


