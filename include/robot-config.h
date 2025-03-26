using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor_group leftChassis;
extern motor_group rightChassis;
extern inertial rightInertial;
extern inertial leftInertial;
extern rotation frontTrackingWheel;
extern rotation backTrackingWheel;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );