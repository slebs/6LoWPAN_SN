################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/sensn/app_fh_com.c \
../src/sensn/app_interface.c \
../src/sensn/app_ping.c \
../src/sensn/perf_test.c 

OBJS += \
./src/sensn/app_fh_com.o \
./src/sensn/app_interface.o \
./src/sensn/app_ping.o \
./src/sensn/perf_test.o 

C_DEPS += \
./src/sensn/app_fh_com.d \
./src/sensn/app_interface.d \
./src/sensn/app_ping.d \
./src/sensn/perf_test.d 


# Each subdirectory must supply rules for building sources it contributes
src/sensn/%.o: ../src/sensn/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -DSENSORNETWORK -DAPP_PING -DCOORDNODE -DF_CPU=8000000UL -DSINGLE_CHIP -DPAN_CHANNEL=20 -DPLATFORM=RCBSINGLE -DUART_DEBUG -DDEBUG=1 -DSERIAL=1 -DIPV6LOWPAN=1 -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega128rfa1 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


