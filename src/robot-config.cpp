#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftChassisMotorA = motor(PORT3, ratio6_1, false);
motor leftChassisMotorB = motor(PORT4, ratio6_1, false);
motor leftChassisMotorC = motor(PORT2, ratio6_1, false);
motor_group leftChassis = motor_group(leftChassisMotorA, leftChassisMotorB,leftChassisMotorC);
motor rightChassisMotorA = motor(PORT5, ratio6_1, true);
motor rightChassisMotorB = motor(PORT6, ratio6_1, true);
motor rightChassisMotorC = motor(PORT7, ratio6_1, true);
motor_group rightChassis = motor_group(rightChassisMotorA, rightChassisMotorB,rightChassisMotorC);
inertial rightInertial = inertial(PORT9);
inertial leftInertial = inertial(PORT10);
rotation backTrackingwheel = rotation(PORT11,false);
rotation frontTrackingwheel = rotation(PORT12,false);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}