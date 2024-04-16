#include "Hbridge.h"

Hbridge::Hbridge()
{
}

void Hbridge::setup(uint8_t pin[], uint8_t channel[])
{
    in[0].setup(pin[0], channel[0], 25000, 10);
    in[1].setup(pin[1], channel[1], 25000, 10);
}

void Hbridge::setSpeed(float duty)
{
    if (duty > 0)
    {
        in[0].setDuty(0);
        in[1].setDuty(duty);
    }
    else if (duty < 0)
    {
        in[0].setDuty(fabs(duty));
        in[1].setDuty(0);
    }
    else{
        in[0].setDuty(0);
        in[1].setDuty(0);
    }
}

Hbridge::~Hbridge()
{
}