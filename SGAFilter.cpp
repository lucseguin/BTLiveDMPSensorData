#include "SGAFilter.hpp"
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
const int16_t sga_coefficients[][9]={
    {0,    0, -3, 12, 17, 12, -3,  0, 0},  //degree 2 or 3 & window size 5
    {0,   -2,  3,  6,  7,  6,  3, -2, 0},  //degree 2 or 3 & window size 7
    {-21, 14, 39, 54, 59, 54, 39, 14, -21},//degree 2 or 3 & window size 9
    {0,   5, -30,  75, 131,  75, -30,   5,  0},//degree 4 or 5 & window size 7
    {15, -55, 30, 135, 179, 135,  30, -55, 15},//degree 4 or 5 & window size 9
};
const int16_t sga_normalisation[]= {35, 21, 231, 231, 429};
uint8_t sga_mid = 4;

SGAFilter::SGAFilter(uint8_t polynomialDegree, uint8_t windowSize) {
    _windowSize = windowSize;
    _polynomialDegree = polynomialDegree;
    
    switch(polynomialDegree) {
        case 2:
        case 3:
            
            if(windowSize == 5)
                _index = 0;
            else if(windowSize == 7)
                _index = 1;
            else if(windowSize == 9)
                _index = 2;
            else {
                Serial.println("Invalid windowSize SGA parameter for polynomialDegree 2 or 3!");
                while (1) {}
            }
            
            break;
        case 4:
        case 5:
            if(windowSize == 7)
                _index = 3;
            else if(windowSize == 9)
                _index = 4;
            else {
                Serial.println("Invalid windowSize SGA parameter for polynomialDegree 4 or 5!");
                while (1) {}
            }
            break;
        default:
            Serial.println("Invalid polynomialDegree SGA parameter!");
            while (1) {}
            break;
    }
    _middle = (windowSize-1)/2;
    _offset = (9-windowSize)/2;
    _history = (uint16_t *)calloc(windowSize, sizeof(uint16_t));
}

int16_t SGAFilter::filter(int16_t current_value)
{
    int64_t sum=0;
    uint8_t i;
    
    //shift all history to the left
    for(i=0;i<_windowSize-1;i++)
    {
        _history[i]=_history[i+1];
    }
    _history[_windowSize-1]=current_value; //append at the end
    
    
    for(i=0;i<_windowSize;i++)
    {
        sum+=_history[i]*sga_coefficients[_index][i+_offset];
    }
    
    _history[_middle] = sum/sga_normalisation[_index];
    return _history[_middle] ;
    
    //    return sum/sga_normalisation[filter->index];;
}
