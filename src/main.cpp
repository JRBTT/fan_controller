#include <util/delay.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "bit.h"
#include "thermistor.h"

#define FOSC 16000000 // Frequency Oscillator 16Mhz for Uno R3
#define BAUD 9600 // 9600 Bits per second
#define MYUBRR FOSC / 16 / BAUD - 1 // My USART Baud Rate Register

volatile bool adcReady = false; 
volatile int adcResult = 0;

// for input capture of fan
volatile float T1 = 0.0;
volatile float T2 = 0.0;
volatile float T_high = 0.0;
volatile float T_low = 0.0;
volatile float freq = 0.0;
// for analog comparator for pump
volatile float T12 = 0.0;
volatile float T22 = 0.0;
volatile float T_high2 = 0.0;
volatile float T_low2 = 0.0;
volatile float freq2 = 0.0;

volatile bool interruptTriggered = false;

void setAdcbit(){
  // ADC initialization for reading the thermistor
  ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << MUX2); //AVCC with external capacitor at AREF pin ADC5 as input
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); 
  // ADPS is prescaler bits 128 prescaler
  // ADEN is ADC Enable
  // ADIE is ADC Interrupt Enable
  DIDR0 = (1 << ADC5D);
  // disabling digital input buffer to reduce power consumption
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

ISR(TIMER1_CAPT_vect) {
  float capturedValue2 = ICR1;

  if (!(TCCR1B & (1 << ICES1))) {
    // Falling edge
    T12 = capturedValue2;
    TCCR1B ^= (1 << ICES1); // Toggle the edge detection
    T_low = T12 - T22;
  } else {
    // Rising edge
    T22 = capturedValue2;
    TCCR1B ^= (1 << ICES1); // Toggle the edge detection
    T_high2 = T22 - T12;
  }
}

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

void setupSensors(){
  //Fast PWM with OCR2A as TOP

  TCCR2A = (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << WGM22);
  TCCR2A |= (1 << COM2B1); // non-inverting mode
  OCR2A = 79; // 25kHz
  OCR2B = 20; //default 25% duty cycle

  // Input Capture setup
  TCCR1B |= (1 << ICES1); // Capture on rising edge
  bitClear(DDRB, PB0); // Set ICP1 (PB0) as input
  TIMSK1 |= (1 << ICIE1); // Enable Input Capture interrupt

  // Analog Comparator setup
  bitClear(DDRD, PD7); // negative
  bitClear(DDRD, PD6); // positive reference 3.3v

  ACSR = (0 << ACD) | (1 << ACBG) | (1 << ACIS1) | (1 << ACIS0); // Bandgap reference, rising edge
  ACSR |= (1 << ACIE); // Enable Analog Comparator interrupt
  TCCR2B |= (1 << CS21); // prescaler 8 (2MHz) 
  TCCR1B |= (1 << CS12); // prescaler 256 (62.5kHz) with overflow at1.04857s

}

ISR(INT0_vect) {
  // Handle the interrupt
  interruptTriggered = true;
}

void setup()
{

  DDRD &= ~(1 << PD2);
  EICRA |= (1 << ISC01); // Set ISC01
  EICRA &= ~(1 << ISC00); // Clear ISC00
  EIMSK |= (1 << INT0); // Enable INT0

  DDRD |= (1 << PD3); // PWM output

  usart_init(MYUBRR); // 103-9600 bps; 8-115200
  setupSensors();
  setAdcbit(); // set ADC5 as input
  sei(); // enable global interrupts
  // Set PB1, PB2, PB3, PB4 as outputs
  DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB4) | (1 << PB5); // PB5 is alarm
  // PORTB |= (1 << PB1) | (1 << PB2) | (1 << PB4);
  DDRD |= (1 << PD5);
  PORTB |= (1 << PB1);
  bitClear(DDRD, PD4); 
  bitSet(PORTD, PD4); // pullup


  updateADC();
}

void alarm(){
  // Make the buzzer sound at a frequency of 1kHz
  while(1){
    for (int i = 0; i < 1000; i++) {
      // Turn the buzzer on and off to create a square wave
      PORTB |= (1 << PB5);
      _delay_us(500); // 500us high
      PORTB &= ~(1 << PB5);
      _delay_us(500); // 500us low
    }
    _delay_ms(1000);
  }
}

float convertToRPM(float frequency){
  float rpm = (frequency / 2)* 60;
  return rpm;
}

int main()
{
    setup();
    
    float temp;
    int previous_button1 = 1;
    int i = 0;
    bool automatic = false;
    while(1)
    {

      // switch device on or off
      int current_button1 = bitRead(PIND, PIND4);
      if (current_button1 != previous_button1 && current_button1 == 0) {
          _delay_ms(50);
          if (bitRead(PIND, PIND4) == current_button1) {
            for(int j = 0; j < 2; j++){
              switch(i) {
                case 0:
                  bitInverse(PORTB, PB1); // white light
                  OCR2B = 20; // 25% duty cycle
                  break;
                case 1:
                  bitInverse(PORTB, PB2); // green light
                  OCR2B = 40; // 50% duty cycle
                  break;
                case 2:
                  bitInverse(PORTD, PD5); // yellow light
                  OCR2B = 60; // 75% duty cycle
                  break;
                case 3:
                  bitInverse(PORTB, PB4); // red light
                  OCR2B = 79; // 100% duty cycle
                  break;
                case 4:
                // Automatic mode
                  PORTB ^= (1 << PB1) | (1 << PB2) | (1 << PB4); // all lights on
                  PORTD ^= (1 << PD5); // all lights on
                  automatic = !automatic; // follows the temperature sensor
              }
              if (j != 1){
                i++;
              }
              if (i > 4) {
                i = 0;
              }
            }
          }
      }
      previous_button1 = current_button1;

      if (adcReady){
          usart_tx_string(">Temperature C: ");
          temp = getTemperature(adcResult);
          usart_tx_float(temp, 3, 2);
          usart_transmit('\n');
          adcReady = false;
          if (automatic && temp > 0.0){
            if (temp > 55.0) {
              OCR2B = 79;
            }
            else{
              float temp2 = temp - 25.0; // Between 25 degrees and 55 degrees 0 - 100% duty cycle
              float dutyCycle = temp2 / 30.0;
              OCR2B = (int)(dutyCycle * 79);
            }
          }
          updateADC();
      }
      if (T_high + T_low > 0.0){
        float time = (T_high + T_low) * 0.000016;
        freq = 1 / time;
        usart_tx_string(">pump rpm: ");
        float rpm = convertToRPM(freq);
        usart_tx_float(rpm, 6, 3);
        usart_transmit('\n');
      }

      if (T_high2 + T_low2 > 0.0){
        float time2 = (T_high2 + T_low2) * 0.000016;
        freq2 = (1 / time2)/2;
        usart_tx_string(">fan rpm: ");
        float rpm2 = convertToRPM(freq2);
        usart_tx_float(rpm2, 6, 3);
        usart_transmit('\n');
      }
      if (interruptTriggered){
        break;
      }
    }
    usart_tx_string("Power failure: Pump");
    alarm();

}