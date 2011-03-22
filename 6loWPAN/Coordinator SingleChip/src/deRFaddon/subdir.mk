################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/deRFaddon/bmm.c \
../src/deRFaddon/data.c \
../src/deRFaddon/deRFapplication.c \
../src/deRFaddon/hdlc_light.c \
../src/deRFaddon/io_access.c \
../src/deRFaddon/link_quality.c \
../src/deRFaddon/queue.c \
../src/deRFaddon/status.c \
../src/deRFaddon/uart.c \
../src/deRFaddon/usb.c \
../src/deRFaddon/util.c 

OBJS += \
./src/deRFaddon/bmm.o \
./src/deRFaddon/data.o \
./src/deRFaddon/deRFapplication.o \
./src/deRFaddon/hdlc_light.o \
./src/deRFaddon/io_access.o \
./src/deRFaddon/link_quality.o \
./src/deRFaddon/queue.o \
./src/deRFaddon/status.o \
./src/deRFaddon/uart.o \
./src/deRFaddon/usb.o \
./src/deRFaddon/util.o 

C_DEPS += \
./src/deRFaddon/bmm.d \
./src/deRFaddon/data.d \
./src/deRFaddon/deRFapplication.d \
./src/deRFaddon/hdlc_light.d \
./src/deRFaddon/io_access.d \
./src/deRFaddon/link_quality.d \
./src/deRFaddon/queue.d \
./src/deRFaddon/status.d \
./src/deRFaddon/uart.d \
./src/deRFaddon/usb.d \
./src/deRFaddon/util.d 


# Each subdirectory must supply rules for building sources it contributes
src/deRFaddon/%.o: ../src/deRFaddon/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -DSENSORNETWORK -DAPP_PING -DCOORDNODE -DF_CPU=8000000UL -DSINGLE_CHIP -DPAN_CHANNEL=20 -DPLATFORM=RCBSINGLE -DUART_DEBUG -DDEBUG=1 -DSERIAL=1 -DIPV6LOWPAN=1 -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega128rfa1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


