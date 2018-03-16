// Invert.h -- Code needed to perform gradient-based inverse problem (design) of a 2D Warren Truss
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

#ifndef INVERT_H
#define INVERT_H 1


//Using SDL and standard IO
#include <SDL.h>
#include <dlib/optimization.h>
#include <stdio.h>
#include <iostream>
#include "Tetrahelix.hpp"
#include "Obstacle.hpp"

#include <math.h>

using namespace std;
using namespace dlib;

// This is kludge -- not gettin find_min to work very well.
// WARNING: This is probably a memory leak

extern double* best_distances;
extern double best_score;



class Invert {
public:

  Tetrahelix *an;
  static Tetrahelix *global_truss;  
  Invert();
  
  // This function weights how close our values are to the goals.
  // At present, this is literally of the unweighted sum of the distances,
  // or th l2_norms between each goal node and its corresponding position.
  // For the purposes of the paper I will call this the "simple distance score" or sds.
  double operator() ( const column_vector& ds) const;

  void set_global_truss(Obstacle obstacle);
  // This is currently computing the sum of the l2_norm of the goal point quandrances (square of distance).
  static double objective(const column_vector& ds);

  // compute the derivatives of the objective as the configuration ds.
  // NOTE: At present this only works with ONE goal node
  static column_vector derivative(const column_vector& ds);

  Obstacle ob;
  
};

#endif
