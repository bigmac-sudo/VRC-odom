#ifndef CONTROL_H
#define CONTROL_H

#include "vex.h"
#include <vector>

// PID class definition
class PID {
public:
    double Kp, Ki, Kd;
    double previousError;
    double integral;

    PID(double Kp, double Ki, double Kd);

    double compute(double error);
};

// Control class to manage the robot's speed based on distance
class Control {
private:
    PID distancePID;  // PID controller for controlling distance
    PID turningPID;
    double targetX, targetY;  // Target position
    double currentX, currentY;  // Current position
    double targetHeading;
    double currentHeading;

public:
    // Control(double Kp, double Ki, double Kd);  // Constructor to initialize PID
    Control(double Kp_dist, double Ki_dist, double Kd_dist, 
            double Kp_turn, double Ki_turn, double Kd_turn); // Updated constructor
    // void setTarget(double x, double y);  // Set target position
    // void updatePosition();  // Update current position
    // void controlSpeed();  // Control robot speed based on distance to target
    // double distanceToTarget();  // Calculate the distance to target
        void setTarget(double x, double y);
    void updatePosition();
    void updateHeading();
    double distanceToTarget();
    double headingError();
    void controlSpeed(int direction);
    void turnToHeading(double targetHeading);
};

#endif
