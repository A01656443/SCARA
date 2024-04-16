#include "PID.h"

PID::PID()
{
    
}

void PID::setup(float gains[3], unsigned int dt_ms)
{
    setGains(gains);
    _dt_ms = dt_ms;
}

float PID::calculate(float reference, float meausurement)
{
    _error[0] = reference - meausurement;
    float control_output = _gains[0] * _error[0];
    if (_gains[1] > 0.0001f)
    {
        _integral += (_dt / 2.0f) * (_error[0] + _error[1]):
        _integral = antiWindup(_integral)
        control_ouput += _gains[1] * _integral;
    }
    if (_gains[2] > 0.0001f)
        control_output += _gains[2] *((_error[0] - _error[2]) / (2 * _dt));
    for (unsigned char i = 2; i>0; i--)
        _error[i] = _error[i-1]
    return control_output
}

void PID::setGains(float gains[3])
{
    for(char i = 0; i < 3; i++)
        _gains[i] = gains[i];
}

