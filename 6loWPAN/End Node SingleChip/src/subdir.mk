################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/avr_sixlowpan.c \
../src/avr_sixlowpan_application.c \
../src/avr_timer.c \
../src/hal_avr.c \
../src/mac.c \
../src/mac_associate.c \
../src/mac_beacon.c \
../src/mac_data.c \
../src/mac_event.c \
../src/mac_route.c \
../src/mac_scan.c \
../src/mac_start.c \
../src/main.c \
../src/radio.c \
../src/rum_application.c \
../src/sensors.c \
../src/serial.c \
../src/sleep.c 

OBJS += \
./src/avr_sixlowpan.o \
./src/avr_sixlowpan_application.o \
./src/avr_timer.o \
./src/hal_avr.o \
./src/mac.o \
./src/mac_associate.o \
./src/mac_beacon.o \
./src/mac_data.o \
./src/mac_event.o \
./src/mac_route.o \
./src/mac_scan.o \
./src/mac_start.o \
./src/main.o \
./src/radio.o \
./src/rum_application.o \
./src/sensors.o \
./src/serial.o \
./src/sleep.o 

C_DEPS += \
./src/avr_sixlowpan.d \
./src/avr_sixlowpan_application.d \
./src/avr_timer.d \
./src/hal_avr.d \
./src/mac.d \
./src/mac_associate.d \
./src/mac_beacon.d \
./src/mac_data.d \
./src/mac_event.d \
./src/mac_route.d \
./src/mac_scan.d \
./src/mac_start.d \
./src/main.d \
./src/radio.d \
./src/rum_application.d \
./src/sensors.d \
./src/serial.d \
./src/sleep.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wl,-u,vfprintf -lprintf_flt -lm -DSENSORNETWORK -DAPP_PING -DENDNODE -DF_CPU=8000000UL -DSINGLE_CHIP -DPAN_CHANNEL=20 -DPLATFORM=RCBSINGLE -DUART_DEBUG -DDEBUG=1 -DSERIAL=1 -DIPV6LOWPAN=1 -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega128rfa1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


