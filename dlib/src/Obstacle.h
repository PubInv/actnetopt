#ifndef OBSTACLE_H
#define OBSTACLE_H 1

#include <stdio.h>
#include <iostream>
#include <dlib/optimization.h>
#include <dlib/geometry.h>

using namespace std;
using namespace dlib;

// ----------------------------------------------------------------------------------------

// In dlib, the general purpose solvers optimize functions that take a column
// vector as input and return a double.  So here we make a typedef for a
// variable length column vector of doubles.  This is the type we will use to
// represent the input to our objective functions which we will be minimizing.
typedef matrix<double,0,1> column_vector;

double distance_2d(column_vector a, column_vector b);


class Obstacle {
public:
  column_vector center;
  double radius;
  // return the objective function contribution for a node at x for this obstacle
  double f(column_vector x);
  // partial derivative of contribution to the objective function for this obstable for this node
  double partial(double d);
};

#endif
