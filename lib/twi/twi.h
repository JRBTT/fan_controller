#ifndef TWI_H
#define TWI_H

#include <stdint.h>

void twi_init_master();
void twi_init_slave();
void twi_start();
void twi_stop();
void twi_write(unsigned char data);
uint8_t twi_read_nack();

#endif // TWI_H