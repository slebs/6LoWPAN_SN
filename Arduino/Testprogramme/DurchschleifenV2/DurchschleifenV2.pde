/*
* Name:  DurchschleifenV2
* Autor: Kevin Pluch
*
* Forwards data from PC to RCB and RCB to PC over Arduino
*/


//------------------------------------------------------------------------
//|                                INCLUDES                              |
//------------------------------------------------------------------------
#include <NewSoftSerial.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// create SoftwareSerial
NewSoftSerial rcbUART(4,5);

// calculate BAUD Rate
#define UART_BAUD(rate) ((F_CPU + rate * 8UL) / (16UL * rate) - 1UL)


//------------------------------------------------------------------------
//|                               SUBROUTINES                            |
//------------------------------------------------------------------------

// initialize USART Interface of Arduino
void USART_Init( unsigned int ubrr)
{
  // enable double speed
  UCSR0A &= (0<<U2X0);                              

  // set the baudrate register
  UBRR0H = 0;
  UBRR0L = (UART_BAUD(ubrr)) & 0xFF;                

  // enable Receiver and Transmitter and RX Interrupt
  UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);  
  // 8 Data, No Parity, 1 Stop bit
  UCSR0C = (3<<UCSZ00);                            
}

// transmit subroutine
int uart_putc (char c)
{
  // wait for empty tx buffer
  while ( !(UCSR0A & _BV(UDRE0)) );      
  // Start transmittion  
  UDR0 = c;          
  // return the char  
  return c;                                        
}

//------------------------------------------------------------------------
//|                               RX INTERRUPT                           |
//------------------------------------------------------------------------
ISR(USART_RX_vect){
  // forward bytes from PC to RCB
  rcbUART.print(UDR0,BYTE);
}

//------------------------------------------------------------------------
//|                                  INITS                               |
//------------------------------------------------------------------------
void setup(){
  USART_Init(9600);
  rcbUART.begin(9600);
  sei();
}

//------------------------------------------------------------------------
//|                               FOREVER LOOP                           |
//------------------------------------------------------------------------
void loop(){
  
  // forward bytes from RCB to PC
  if(rcbUART.available()){
    uart_putc(rcbUART.read());  
  }
}
