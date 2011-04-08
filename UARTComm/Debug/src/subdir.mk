################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/TWI_Master.c \
../src/hdlc_light.c \
../src/io_access.c \
../src/main.c \
../src/tmp75.c \
../src/twimaster.c \
../src/uart.c 

OBJS += \
./src/TWI_Master.o \
./src/hdlc_light.o \
./src/io_access.o \
./src/main.o \
./src/tmp75.o \
./src/twimaster.o \
./src/uart.o 

C_DEPS += \
./src/TWI_Master.d \
./src/hdlc_light.d \
./src/io_access.d \
./src/main.d \
./src/tmp75.d \
./src/twimaster.d \
./src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega128rfa1 -DF_CPU=8000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


