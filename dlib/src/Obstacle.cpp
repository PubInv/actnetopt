// Obstacle.cpp -- Code needed to implmenet gradient-based obstacle
// Copyright (C) Robert L. Read, 2018
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "Obstacle.hpp"

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
  double d = distance_2d(x,center);
  return weight *  (1.0/(1.0 + d*d) + 1)*H(radius-d);
}

  // partial derivative of contribution to the objective function for this obstable for this node
double Obstacle::partial(double d) {
  double rmd = radius - d;
  double p =
    -(1.0 / (pow(d+1.0,2)) + 1.0) * dirac(rmd) -
    2*H(rmd)/pow(d+1.0,3);
  return p*weight;
}
