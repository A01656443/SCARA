#ifndef __PID_H__
#define __PID_H__

class PID
{
public:
    PID(); 
    void setup(float gains[3], unsigned int dt_ms, float integral_saturation = 10);
    float calculate(float reference, float measurement); 
    void setGains(float gains[3]);
    float getError();
    
private:
    float antiWindup(float integral);
    float _gains[3];
    unsigned int _dt_ms;
    float _error[3];
    float _saturation;
    float _integral = 0;
};
#endif // __PID_H__