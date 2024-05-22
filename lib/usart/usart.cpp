#include "usart.h"
#include <avr/io.h> // Include here if avr/io.h is needed for USART functions
#include "bit.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

// Set the baud rate
void usart_init(unsigned int ubrr)
{
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  // enable receiver and transmitter
  UCSR0B = (1 << TXEN0) | (1 << RXEN0);
  // set frame format: USBS0 = 0b1 2stop bit, UCSZ00 = 0b11 8bit data,
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

void usart_transmit(unsigned char data)
{
  /* Wait for empty transmit buffer */
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  /* Put data into buffer, sends the data */
  UDR0 = data;
}

void usart_tx_string(const char *pStr)
{
  while (*pStr != '\0')
  {
    usart_transmit(*pStr);
    pStr++;
  }
}

void usart_tx_float(float x, char num_digits_int, char num_digits_decimal)
{
  char num_elements = num_digits_int + num_digits_decimal + 1 + 1; // one decimal point and one null terminator
  char buffer[num_elements];

  dtostrf(x, num_elements - 1, num_digits_decimal, buffer);
  buffer[num_elements - 1] = '\0';
  usart_tx_string(buffer);
}


void usart_tx_reg(uint8_t data) 
{
  char buffer[4]; // Buffer to hold the string. 3 digits + null terminator
  sprintf(buffer, "%u", data);
  usart_tx_string(buffer);
}