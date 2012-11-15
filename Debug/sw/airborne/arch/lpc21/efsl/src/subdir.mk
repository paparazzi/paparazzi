################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sw/airborne/arch/lpc21/efsl/src/debug.c \
../sw/airborne/arch/lpc21/efsl/src/dir.c \
../sw/airborne/arch/lpc21/efsl/src/disc.c \
../sw/airborne/arch/lpc21/efsl/src/efs.c \
../sw/airborne/arch/lpc21/efsl/src/extract.c \
../sw/airborne/arch/lpc21/efsl/src/fat.c \
../sw/airborne/arch/lpc21/efsl/src/file.c \
../sw/airborne/arch/lpc21/efsl/src/fs.c \
../sw/airborne/arch/lpc21/efsl/src/ioman.c \
../sw/airborne/arch/lpc21/efsl/src/ls.c \
../sw/airborne/arch/lpc21/efsl/src/mkfs.c \
../sw/airborne/arch/lpc21/efsl/src/partition.c \
../sw/airborne/arch/lpc21/efsl/src/plibc.c \
../sw/airborne/arch/lpc21/efsl/src/time.c \
../sw/airborne/arch/lpc21/efsl/src/ui.c 

OBJS += \
./sw/airborne/arch/lpc21/efsl/src/debug.o \
./sw/airborne/arch/lpc21/efsl/src/dir.o \
./sw/airborne/arch/lpc21/efsl/src/disc.o \
./sw/airborne/arch/lpc21/efsl/src/efs.o \
./sw/airborne/arch/lpc21/efsl/src/extract.o \
./sw/airborne/arch/lpc21/efsl/src/fat.o \
./sw/airborne/arch/lpc21/efsl/src/file.o \
./sw/airborne/arch/lpc21/efsl/src/fs.o \
./sw/airborne/arch/lpc21/efsl/src/ioman.o \
./sw/airborne/arch/lpc21/efsl/src/ls.o \
./sw/airborne/arch/lpc21/efsl/src/mkfs.o \
./sw/airborne/arch/lpc21/efsl/src/partition.o \
./sw/airborne/arch/lpc21/efsl/src/plibc.o \
./sw/airborne/arch/lpc21/efsl/src/time.o \
./sw/airborne/arch/lpc21/efsl/src/ui.o 

C_DEPS += \
./sw/airborne/arch/lpc21/efsl/src/debug.d \
./sw/airborne/arch/lpc21/efsl/src/dir.d \
./sw/airborne/arch/lpc21/efsl/src/disc.d \
./sw/airborne/arch/lpc21/efsl/src/efs.d \
./sw/airborne/arch/lpc21/efsl/src/extract.d \
./sw/airborne/arch/lpc21/efsl/src/fat.d \
./sw/airborne/arch/lpc21/efsl/src/file.d \
./sw/airborne/arch/lpc21/efsl/src/fs.d \
./sw/airborne/arch/lpc21/efsl/src/ioman.d \
./sw/airborne/arch/lpc21/efsl/src/ls.d \
./sw/airborne/arch/lpc21/efsl/src/mkfs.d \
./sw/airborne/arch/lpc21/efsl/src/partition.d \
./sw/airborne/arch/lpc21/efsl/src/plibc.d \
./sw/airborne/arch/lpc21/efsl/src/time.d \
./sw/airborne/arch/lpc21/efsl/src/ui.d 


# Each subdirectory must supply rules for building sources it contributes
sw/airborne/arch/lpc21/efsl/src/%.o: ../sw/airborne/arch/lpc21/efsl/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


