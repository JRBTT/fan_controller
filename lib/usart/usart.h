#ifndef USART_H
#define USART_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

void usart_init(unsigned int ubrr);
void usart_transmit(unsigned char data);
void usart_tx_string(const char *pStr);
void usart_tx_float(float x, char num_digits_int, char num_digits_decimal);
void usart_tx_reg(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif // USART_H
