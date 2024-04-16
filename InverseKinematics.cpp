#include "InverseKinematics.h"
#include <math.h>

#define EPSILON 0.0001 // Pequeña cantidad para evitar divisiones por cero

InverseKinematics::InverseKinematics() 
{
}

void InverseKinematics::setup(float l1, float l2, float l3, float l4) {
    L1 = l1;
    L2 = l2;
    L3 = l3;
    L4 = l4;
}

bool InverseKinematics::calculate(float x, float y, float z, float roll, float& theta1, float& d2, float& d3, float& theta4) {
    // Calculating theta1 (base rotation)
    theta1 = atan2(y, x);

    // Calculating distance to wrist center
    float r_wc = sqrt(x * x + y * y);

    // Calculating distance to wrist point (z coordinate)
    float z_wc = z - L4 * sin(roll);

    // Calculating the length of the wrist center projection on the XY plane
    float r_proj_wc = sqrt(r_wc * r_wc - (z_wc * z_wc));

    // Calculating theta4 (wrist rotation)
    theta4 = atan2(z_wc, r_proj_wc) - roll;

    // Calculating d3 (prismatic link 3)
    float a = r_proj_wc - L1;
    float b = z_wc;
    d3 = sqrt(a * a + b * b) - L2;

    // Calculating d2 (prismatic link 2)
    d2 = L3 - r_wc;
    
    // Check if the point is within the workspace
    if (z < 0 || r_wc < L1 || r_wc > L1 + L2 + L3 || d2 < 0 || d3 < 0) {
        return false; // Point outside of workspace
    }

    return true;
}
