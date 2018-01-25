#ifndef INVERT_H
#define INVERT_H 1


//Using SDL and standard IO
#include <SDL.h>
#include <dlib/optimization.h>
#include <stdio.h>
#include <iostream>
#include "TriLadder.h"

#include <math.h>

#define PI 3.14159265

using namespace std;
using namespace dlib;

// This is kludge -- not gettin find_min to work very well.
// WARNING: This is probably a memory leak

extern double* best_distances;
extern double best_score;



class Invert {
public:

  TriLadder *an;
  static TriLadder *cur_an;  
  Invert();
  
  // This function weights how close our values are to the goals.
  // At present, this is literally of the unweighted sum of the distances,
  // or th l2_norms between each goal node and its corresponding position.
  // For the purposes of the paper I will call this the "simple distance score" or sds.
  double operator() ( const column_vector& ds) const;

  void set_cur_an();

  

  // This is currently computing the sum of the l2_norm of the goal point quandrances (square of distance).
  static double objective(const column_vector& ds);

  // compute the derivatives of the objective as the configuration ds.
  // NOTE: At present this only works with ONE goal node
  static column_vector derivative(const column_vector& ds);
  
};

#endif
