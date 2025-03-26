#include "control.h"
#include <cmath>
#include "vex.h"
#include "Odom.h"
Odometry Odometry;
// PID class constructor
double currentHeading = 0;
PID::PID(double Kp, double Ki, double Kd)
    : Kp(Kp), Ki(Ki), Kd(Kd), previousError(0), integral(0) {}

// PID compute method for calculating control signal
double PID::compute(double error) {
    integral += error;  // Sum of errors (integral term)
    double derivative = error - previousError;  // Change in error (derivative term)
    
    // Proportional term
    double output = Kp * error + Ki * integral + Kd * derivative;
    
    // Save current error for next cycle
    previousError = error;
    
    return output;
}
void Control::updateHeading() {
    currentHeading = Odometry.inertialHeading(); // Assume this function exists in Odometry
}
double Control::headingError() {
    double desiredHeading = atan2(targetY - currentY, targetX - currentX) * (180.0 / M_PI); // Convert to degrees
    double error = desiredHeading - currentHeading;

    // Normalize error to be between -180 and 180 degrees
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    return error;
}

// Control class constructor
Control::Control(double Kp_dist, double Ki_dist, double Kd_dist, 
                 double Kp_turn, double Ki_turn, double Kd_turn)
    : distancePID(Kp_dist, Ki_dist, Kd_dist), 
      turningPID(Kp_turn, Ki_turn, Kd_turn),  // Initialize turning PID
      targetX(0), targetY(0), currentX(0), currentY(0), 
      targetHeading(0), currentHeading(0) {} // Initialize heading values


// Set target position
void Control::setTarget(double x, double y) {
    targetX = x;
    targetY = y;
}

// Update current position
void Control::updatePosition() {
    currentX = Odometry.newPos()[0];
    currentY = Odometry.newPos()[1];
}

// Method to calculate the distance from the robot to the target position
double Control::distanceToTarget() {
    double dx = targetX - currentX;
    double dy = targetY - currentY;
    return sqrt(dx * dx + dy * dy);  // Use Pythagorean theorem
}
 
// Control speed based on distance to the target
void Control::controlSpeed(int direction) {
    double distanceError = distanceToTarget();  // Distance error
    double turnError = headingError();         // Heading error

    double distanceSpeed = distancePID.compute(distanceError); // Compute forward speed
    double turnSpeed = turningPID.compute(turnError); // Compute turning correction

    // Modify left and right speeds for turning control
    double leftSpeed = distanceSpeed - turnSpeed;  // Left wheel slows down when turning right
    double rightSpeed = distanceSpeed + turnSpeed; // Right wheel slows down when turning left

    // Limit speed values
    double maxSpeed = 200;
    leftSpeed = std::max(std::min(leftSpeed, maxSpeed), -maxSpeed);
    rightSpeed = std::max(std::min(rightSpeed, maxSpeed), -maxSpeed);

//1 is forward, -1 is reverse, apply speed to motors
if (direction == 1){
      //forward
    leftChassis.spin(forward, leftSpeed, velocityUnits::rpm);
    rightChassis.spin(forward, rightSpeed, velocityUnits::rpm);
} else if (direction == -1){
  //reverse
    leftChassis.spin(reverse, leftSpeed, velocityUnits::rpm);
    rightChassis.spin(reverse, rightSpeed, velocityUnits::rpm);
}
}

void Control::turnToHeading(double targetHeading) {
    double turnError = targetHeading - currentHeading;  // Calculate heading error

    // Normalize error to be between -180 and 180 degrees
    while (turnError > 180) turnError -= 360;
    while (turnError < -180) turnError += 360;

    // Apply PID controller until the error is small enough
    while (std::abs(turnError) > 1.0) {  // Keep turning until error is small (1 degree)
        updateHeading();  // Update the current heading
        turnError = targetHeading - currentHeading;

        // Normalize error again after updating heading
        while (turnError > 180) turnError -= 360;
        while (turnError < -180) turnError += 360;

        // Calculate turn speed with PID
        double turnSpeed = turningPID.compute(turnError);

        // Apply turn speed to motors (left and right wheels)
        leftChassis.spin(forward, -turnSpeed, velocityUnits::rpm);  // Negative for left turn
        rightChassis.spin(forward, turnSpeed, velocityUnits::rpm);  // Positive for right turn

        vex::task::sleep(20);  // Prevent CPU overload and give time for motors to adjust
    }

    // Stop motors once the turn is complete
    Odometry.Stop();
}


