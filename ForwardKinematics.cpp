#include "ForwardKinematics.h"

ForwardKinematics::ForwardKinematics()
{
}

void ForwardKinematics::setup(float DH_parameters[][4], uint8_t num_dof)
{
    _DH = DH_parameters;
    _num_dof = num_dof;
}

void ForwardKinematics::updateDH(uint8_t dof, DH parameter, float value)
{
    _DH[dof][parameter] = value;
}

void ForwardKinematics::calculate(float pose[6])
{
    // HM acomula resultados
    float HM_total[][4] = {{1,0,0,0}, 
                           {0,1,0,0}, 
                           {0,0,1,0},
                           {0,0,0,1}};
                           
    // Operación para cada Dof
    for (uint8_t i = 0; i < _num_dof; i++)
    {
        float t = DEG2RAD * _DH[i][0];
        float d = _DH[i][1];
        float a = DEG2RAD *_DH[i][2];
        float r = _DH[i][3];
        float HM_dof [][4] = {{cos(t), -cos(a)*sin(t), sin(a)*sin(t), r*cos(t)},
                            {sin(t), cos(a)*cos(t), -sin(a)*cos(t), r*sin(t)},
                            {0, sin(a), cos(a), d},
                            {0, 0, 0, 1}};

        float HM_row_temp[4];
        for (int row = 0; row < 3; row++)
        {
            for (int column = 0; column < 4; column++)
            {
                HM_row_temp[column] = 0;
                for (int k = 0; k < 4; k++)
                    HM_row_temp[column] += HM_total[row][k] * HM_dof[k][column];
            }
            for (int column = 0; column <4; column++)
                HM_total[row][column] = HM_row_temp[column];
        }
    
    }
    // Convert final HM to pose
    pose[0] = RAD2DEG * atan2(HM_total[1][0], HM_total[0][0]); // RPY Roll
    pose[1] = RAD2DEG * atan2(-HM_total[2][0], sqrt(pow(HM_total[2][1], 2) + pow(HM_total[2][2], 2))); // RPY Pitch
    pose[2] = RAD2DEG * atan2(HM_total[2][1], HM_total[2][2]); // RPY Yaw
    pose[3] = HM_total[0][3]; // X
    pose[4] = HM_total[1][3]; // Y
    pose[5] = HM_total[2][3]; // Z
}
