#include <util/delay.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <avr/interrupt.h>

#include "usart.h"
#include "bit.h"
#include "timer.h"
#include "twi.h"
#include "thermistor.h"

#define FOSC 16000000 // Frequency Oscillator 16Mhz for Uno R3
#define BAUD 9600 // 9600 Bits per second
#define MYUBRR FOSC / 16 / BAUD - 1 // My USART Baud Rate Register
#define MAXPWM 1023
#define MAXDELAY 61 // 1 second

volatile bool adcReady = false; 
volatile int adcResult = 0;

void setAdcbit(){
  // ADC initialization for reading the thermistor
  ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << MUX2);
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
  DIDR0 = (1 << ADC5D);
}

int updateADC(){
  bitSet(ADCSRA, ADSC); // start conversion
}

ISR(ADC_vect) {
    // Handle the ADC conversion result
    adcResult = ADC;
    adcReady = true;
   
    // Do something with result
}

void setupPWM(){
  bitSet(DDRB, PB2);
  // Fast PWM with OCR1A as TOP
  TCCR1A = (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << WGM13) | (1 << WGM12);
  TCCR1A |= (1 << COM1B1); // non-inverting mode
  OCR1A = 79; // 25kHz
  TCCR1B |= (1 << CS11);
  OCR1B = 31;
}

void setup()
{
  usart_init(MYUBRR); // 103-9600 bps; 8-115200
  setAdcbit(); // set ADC5 as input
  sei(); // enable global interrupts
  updateADC();


  // setup PWM
}

int main()
{
    setup();
    setupPWM();
    float temp;

    while(1)
    {
        if (adcReady){
            usart_tx_string(">Temperature: ");
            temp = getTemperature(adcResult);
            usart_tx_float(temp, 3, 2);
            usart_transmit('\n');
            adcReady = false;
            updateADC();
        }
    }
}