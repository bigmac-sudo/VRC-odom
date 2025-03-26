#include "vex.h"
#include "Odom.h"
#include <vector>
#include <cmath>
//initialize variables

//x and y are measured in mm
  double x1 =0;
  double y_1 =0;
  double prevencoder = 0;
  std::vector<double> position = {0,0,0};
float heading;
Odometry Odometry;

void Odometry::stop(){
  leftChassis.stop(brake);
  rightChassis.stop(brake);
}


double Odometry::inertialHeading(){
  return (leftInertial.heading() + rightInertial.heading())/2;
}

std::vector<double> Odometry::newPos(){
  heading = inertialHeading();
  position = {newXvalue(), newYvalue(), inertialHeading()};
  prevencoder = backTrackingWheel.position(rev);
  x1 = newXvalue();
  y_1 = newYvalue();
  return position;
  }
double Odometry::degtorad(double deg){
  return deg*(M_PI/180);
}

double Odometry::distanceTraveled(double rev){
  return rev*220; //assuming 2.75 inch omni wheels, unit is mm
}

double Odometry::newXvalue(){
  return (x1+(distanceTraveled(backTrackingWheel.position(rev)-prevencoder)*cos(inertialHeading())));


}
double Odometry::newYvalue(){
  return (y_1+(distanceTraveled(backTrackingWheel.position(rev)-prevencoder)*sin(inertialHeading())));

}



  // std::vector<double> Odometry::getPos() {
  //   heading  = inertialHeading();
  //   // Create a vector to hold the position values and return it
  //   std::vector<double> position = {x_3, y_3, heading};  // Storing the position values
  //   return position;  // Return the vector
// }







