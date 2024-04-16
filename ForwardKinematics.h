#ifndef __FORWARDKINEMATICS_H__
#define __FORWARDKINEMATICS_H__

#include <math.h>

#define DEG2RAD 0.0174533F
#define RAD2DEG 57.29578F

enum DH
{
    THETA,
    D,
    ALPHA,
    R
};

class ForwardKinematics
{
public:
    ForwardKinematics();
    void setup(float DH_parameters[][4], uint8_t num_dof);
    void updateDH(uint8_t dof, DH parameter, float value);
    void calculate(float pose[6]);

private:
    uint_8 _num_dof;
    float(*_DH)[4];
};

#endif // _FORWARDKINEMATICS_H