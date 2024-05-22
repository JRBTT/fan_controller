// prescaler.cpp
#include "timer.h"
#include <avr/io.h>
#include "bit.h"

void setPrescaler_tc0(int option)
{
  TCCR0B |= ((option & 0x07) << CS00);
}

void setPrescaler_tc1(int option)
{
  TCCR1B |= ((option & 0x07) << CS10);
}

void set_tc0_mode(int mode){
    TCCR0A = 0; //Clear registers
    TCCR0B = 0;
    TCCR0A |= ((mode & 0x03) << WGM00); //first 2 bits
    TCCR0B |= ((mode & 0x04) << WGM02); //thrird bit
}

void set_tc1_mode(int mode){
    TCCR1A = 0; //Clear registers
    TCCR1B = 0;
    TCCR1A |= ((mode & 0x03) << WGM10); //first 2 bits
    TCCR1B |= ((mode & 0x18) << WGM12); //4th and 5th bits
}
    