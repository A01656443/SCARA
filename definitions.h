#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <Arduino.h>
#include <PID.h>

PID pid[3]
int dt_ms = 500;

float pid_gains_1[2][3]={{1,2,3},{3,4,5}}
float pid_gains_2[2][3]={{1,2,3},{3,4,5}}
float pid_gains_3[2][3]={{1,2,3},{3,4,5}}

#endif // __DEFINITIONS_H__

/*
#ifndef __SCARA_CONTROLLER_H__
#define __SCARA_CONTROLLER_H__

#include <Arduino.h>
#include "PID.h"
#include "ForwardKinematics.h"
#include "InverseKinematics.h"

#define MOTOR1_PIN1 9
#define MOTOR1_PIN2 10
#define MOTOR2_PIN1 11
#define MOTOR2_PIN2 12

class SCARA_Controller {
public:
    SCARA_Controller();
    void setup();
    void moveToCoordinates(float x, float y, float z);
    void loop();

private:
    PID pid[2];
    ForwardKinematics fk;
    InverseKinematics ik;

    float current_pose[3]; // Current joint angles
    float target_pose[3]; // Target joint angles
};

#endif // __SCARA_CONTROLLER_H__
