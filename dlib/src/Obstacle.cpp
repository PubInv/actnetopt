#include "Obstacle.h"

#include <stdio.h>
#include <iostream>
#include <dlib/geometry.h>

#include <math.h>


using namespace std;
using namespace dlib;



// This is the Heaviside function...
double H(double x) {
  return (x >= 0.0) ? 1.0 : 0.0;
}
 
// This is the dirac delta function...
double dirac(double d) {
  return d == 0.0 ? 1.0 : 0.0;
}

// return the objective function contribution for a node at x for this obstacle
double Obstacle::f(column_vector x) {
  double f = 0.0;
  double d = distance_2d(x,center);
  f = (1.0/(1.0 + d*d) + 1)*H(radius-d);
  
  return f*weight;
}

  // partial derivative of contribution to the objective function for this obstable for this node
double Obstacle::partial(double d) {
  double rmd = radius - d;
  double p =
    -(1.0 / (pow(d+1.0,2)) + 1.0) * dirac(rmd) -
    2*H(rmd)/pow(d+1.0,3);
  return p*weight;
}
