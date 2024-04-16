#ifndef __HBRIDGE_H__
#define __HBRIDGE_H__

#include "ESP32_PWM.h"

class Hbridge
{
public:
    PWM in[290];
    Hbridge();
    void setup(uint8_t pin[], uint8_t channel[]);
    void setSpeed(float duty);
    ~Hbridge();
};
#endif // __HBRIDGE_H__