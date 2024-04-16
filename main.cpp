#include "definitions.h"
#include "Arduino.h"

void setup() {
    // Comenta todo para que no lo olvides
    // Declara pines del motor1
    pinmode(19, OUTPUT)
    pinmode(20, OUTPUT)
    pid[1].setup(pid_gains_1, dt_ms);
    pid[2].setup(pid_gains_2, dt_ms);
    pid[3].setup(pid_gains_3, dt_ms);

}

void loop() {
    //Reference es el punto al que va
    // measurement es el punto actual
    float velmotor1 = pid[1].calculate()

}

/*//
#include "Definitions.h"

Definitions::Definitions() {
}

void Definitions::setup() {
    // Setup PID controllers
    float pid_gains[2][3] = {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}; // Placeholder gains
    pid[0].setup(pid_gains[0], 10); // Tune PID gains and set appropriate dt_ms
    pid[1].setup(pid_gains[1], 10); // Tune PID gains and set appropriate dt_ms

    // Setup Forward Kinematics
    float dh_parameters[2][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}}; // Placeholder parameters
    fk.setup(dh_parameters, 2);

    // Setup Inverse Kinematics
    ik.setup(10, 10, 10, 10); // Tune link lengths
}

void Definitions::moveToCoordinates(float x, float y, float z) {
    // Calculate inverse kinematics to get joint angles
    ik.calculateInverseKinematics(x, y, z, current_pose[0], current_pose[1], current_pose[2]);

    // Set target pose for PID control
    target_pose[0] = current_pose[0];
    target_pose[1] = current_pose[1];
}

void Definitions::loop() {
    // PID control
    for (int i = 0; i < 2; i++) {
        float control_output = pid[i].calculate(current_pose[i], target_pose[i]);

        // Use control output to adjust motor speed or position
        // Example: analogWrite(MOTOR1_PIN1, control_output);
    }
}
