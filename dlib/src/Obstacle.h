// Obstacle.h -- Implement a gradient-based obstacle
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
  double weight= 1.0;
};

#endif
