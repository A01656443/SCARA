#ifndef __INVERSEKINEMATICS_H__
#define __INVERSEKINEMATICS_H__

#include <Arduino.h>

class InverseKinematics
{ 
public:
    InverseKinematics();
    void setup(float l1, float l2, float l3, float l4); // Configurar longitudes de los eslabones
    bool calculate(float x, float y, float z, float roll, float& theta1, float& d2, float& d3, float& theta4); // Calcular la cinemática inversa

private:
    float L1; // Longitud del primer eslabón (rotativo)
    float L2; // Longitud del segundo eslabón (prismático)
    float L3; // Longitud del tercer eslabón (prismático)
    float L4; // Longitud del cuarto eslabón (prismático)
};

#endif // __INVERSEKINEMATICS_H__
