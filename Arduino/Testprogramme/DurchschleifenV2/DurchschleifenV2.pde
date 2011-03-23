#include <NewSoftSerial.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

NewSoftSerial rcbUART(4,5);

#define UART_BAUD(rate) ((F_CPU) / (16UL * (rate)) - 1UL)

void USART_Init( unsigned int ubrr)
{
   UCSR0A &= (0<<U2X0);                              // enable double speed

   UBRR0L = (UART_BAUD(ubrr)) & 0xFF;           // set the baudrate register
   UBRR0H = 0;

   UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);  // enable Receiver and Transmitter and RX Interrupt
   UCSR0C = (3<<UCSZ00);                            // 8 Data, No Parity, 1 Stop bit
}

int uart_putc (char c)
{
   while ( !(UCSR0A & _BV(UDRE0)) );                // wait for empty tx buffer
   UDR0 = c;                                        // Start transmittion
   return c;                                        // return the char
}

ISR(USART_RX_vect){
  rcbUART.print(UDR0,BYTE);
}

void setup(){
  USART_Init(9600);
  rcbUART.begin(9600);
  sei();
}

void loop(){
    if(rcbUART.available()){
      uart_putc(rcbUART.read());  
    }
}


  
