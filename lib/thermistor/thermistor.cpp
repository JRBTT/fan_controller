#include "thermistor.h"
#include <math.h>

float getTemperature(float analog){
    float voltageDividerR1 = 20000;
    float bValue = 3470;
    float R1 = 10000;
    float T1 = 298.15;
    float euler = 2.718281828;
    float R2;
    float T2;
    float a ;
    float b ;
    float c ;
    float d ;
    float temp;

    R2 = (voltageDividerR1 * analog) / (1023 - analog);
    a = 1/T1;
    b = log10f(R1/R2);
    c = b / log10f(euler);
    d = c / bValue;
    T2 = 1 / (a - d);
    temp = T2 - 273.15;
    return temp;
}