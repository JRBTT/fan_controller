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
volatile float T1 = 0.0;
volatile float T2 = 0.0;
volatile float T_high = 0.0;
volatile float T_low = 0.0;
volatile float freq = 0.0;
volatile uint32_t timerValue = 0;
volatile bool periodCaptured = false;

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

// ISR(TIMER1_CAPT_vect) {
//   float capturedValue = ICR1;

//   if (!(TCCR1B & (1 << ICES1))) {
//     // Falling edge
//     T1 = capturedValue;
//     TCCR1B ^= (1 << ICES1); // Toggle the edge detection
//     T_low = T1 - T2;
//   } else {
//     // Rising edge
//     T2 = capturedValue;
//     TCCR1B ^= (1 << ICES1); // Toggle the edge detection
//     T_high = T2 - T1;
//   }
// }

ISR(ANALOG_COMP_vect) {
  float capturedValue = TCNT1;  // Capture the current value of Timer1

  if (!(ACSR & (1 << ACIS0))) {
    // Falling edge
    T1 = capturedValue;
    ACSR ^= (1 << ACIS0); // Toggle the edge detection
    T_low = T1 - T2;
  } else {
    // Rising edge
    T2 = capturedValue;
    ACSR ^= (1 << ACIS0); // Toggle the edge detection
    T_high = T2 - T1;
  }
}

void setupTach(){
  bitSet(DDRB, PB2);
  // Fast PWM with OCR1A as TOP
  // TCCR1A = (1 << WGM11) | (1 << WGM10);
  // TCCR1B = (1 << WGM13) | (1 << WGM12);
  // TCCR1A |= (1 << COM1B1); // non-inverting mode
  // OCR1A = 79; // 25kHz
  // OCR1B = 40;

  // Input Capture setup
  // TCCR1B |= (1 << ICES1); // Capture on rising edge
  // bitClear(DDRB, PB0); // Set ICP1 (PB0) as input
  // TIMSK1 |= (1 << ICIE1); // Enable Input Capture interrupt

  // Analog Comparator setup
  bitClear(DDRD, PD7); // negative
  bitClear(DDRD, PD6); // positive reference 3.3v

  ACSR = (0 << ACD) | (1 << ACBG) | (1 << ACIS1) | (1 << ACIS0); // Bandgap reference, rising edge
  ACSR |= (1 << ACIE); // Enable Analog Comparator interrupt


  TCCR1B = (1 << CS12); // prescaler 256 (62.5kHz) with overflow at1.04857s

}

void setup()
{
  usart_init(MYUBRR); // 103-9600 bps; 8-115200
  setupTach();
  setAdcbit(); // set ADC5 as input
  sei(); // enable global interrupts
  updateADC();


  // setup PWM
}

int main()
{
    setup();
    
    float temp;

    while(1)
    {
      // if (adcReady){
      //     usart_tx_string(">Temperature: ");
      //     temp = getTemperature(adcResult);
      //     usart_tx_float(temp, 3, 2);
      //     usart_transmit('\n');
      //     adcReady = false;
      //     updateADC();
      // }
      if (T_high + T_low > 0.0){
        float time = (T_high + T_low) * 0.000016;
        freq = 1 / time;
        usart_tx_string(">fan freq: ");
        usart_tx_float(freq, 6, 3);
        usart_transmit('\n');
      }
    }
}