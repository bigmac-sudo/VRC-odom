#ifndef Odom_H
#define Odom_H

#include <vector>
#include "vex.h"

class Odometry {
private:
double heading;
double temp_radians;
  std::vector<double> position;
  double x1;
  double y_1;
  double prevencoder;
public:
    Odometry();
    double inertialHeading();
    void stop();
    std::vector<double> getPos();
    std::vector<double>  newPos();
    void startPos();
    double degtorad(double degrees);
    void goToPoint();
    double distanceTraveled(double rev);
    double newXvalue();
    double newYvalue();
};

#endif