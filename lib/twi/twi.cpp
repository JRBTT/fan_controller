#include "twi.h"
#include <avr/io.h>
#include "bit.h"
#include "usart.h"

void twi_init_master(void) {
    // Set up TWI module for master mode
    TWSR = 0; // Set prescalar for 1
    TWBR = 0x47; // Set bit rate register (see datasheet)
    TWCR = (1<<TWEN); // Enable TWI
}

void twi_start(void) {
    // Send start condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    // Wait for TWINT Flag set. This indicates that the START condition has been transmitted
    while ((TWCR & (1<<TWINT)) == 0);
}

void twi_stop(void) {
    // Transmit STOP condition
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

void twi_write(unsigned char data)
{
    // Load data into TWI data register
    TWDR = data;
    // Clear TWINT to start transmission
    TWCR = (1 << TWINT) | (1 << TWEN);
    // Wait for transmission to complete
    while ((TWCR & (1 << TWINT)) == 0);
}

uint8_t twi_read_nack(void)
{
    // Start TWI read operation and do NOT acknowledge data after reception
    TWCR = (1<<TWINT) | (1<<TWEN) | (0<<TWEA);
    // Wait for TWINT flag set. This indicates that the DATA has been received
    while((TWCR & (1<<TWINT)) == 0);
    // Return received data
    return TWDR;
}