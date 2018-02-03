// Invert.cpp -- Code needed to perform gradient-based inverse problem (design) of a 2D Warren Truss
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

#include "Invert.h"
#include "Obstacle.h"

using namespace std;
using namespace dlib;


// These are globals, which is very unforatunate. However, dlib
// graditent routines do NOT reliably return the best value as the documentation says.
// Therefore these are necessary.
double* best_distances = 0;
double best_score;

Invert::Invert() {

}
  
  double Invert::operator() ( const column_vector& ds) const
  {
    global_truss = an;
    return objective(ds);
  }

  void  Invert::set_global_truss(Obstacle o) {
    global_truss = an;
    global_truss->obstacle = o;
    if (best_distances == 0) {
     best_distances = new double[(global_truss->num_nodes-3)*2 +3];
    }
  }

  // This is currently computing the sum of the l2_norm of the goal point quandrances (square of distance).
  double Invert::objective(const column_vector& ds) {

    if (debug) std::cout << "OBJECTIVE INPUTS" << std::endl;
    for (int i = 0; i < global_truss->var_edges; ++i) {
           if (debug) std::cout << i << " : " << ds(i) << std::endl;
	  global_truss->distance(i+1) = ds(i);
    }

    column_vector *coords = new column_vector[global_truss->num_nodes];

    // If I don't change an here, I'm not changing the coords!!
    find_all_coords(global_truss,coords);
    
    for(int i = 0; i < global_truss->num_nodes; i++) {
      //       std::cout << " coords["<< i << "]" << coords[i](0) << "," << coords[i](1) << std::endl;
    }
    
    double v = 0.0;
    for(int i = 0; i < global_truss->goal_nodes.size(); i++) {
      int idx = global_truss->goal_nodes[i];
      if (debug) std::cout << "idx " <<  idx  << std::endl;      
      column_vector g = global_truss->goals[i];
      column_vector c = coords[idx];
      column_vector x(2);
      x(0) = g(0);
      x(1) = g(1);
      column_vector y(2);
      y(0) = c(0);
      y(1) = c(1);
      
      column_vector d(2);

      d = x - y;
      
      if (debug) std::cout << "Invert d " <<  d(0) <<  "," << d(1) <<  " " << std::endl;

      v += (global_truss->goal_weights[i]*distance_2d(x,y));
    }

    // now we must add in the increase to the object caused by the obstacle!!
    for(int j = 0; j < global_truss->num_nodes; j++) {
      // Now, does the partial derivative of this node exist?
      double di = distance_2d(coords[j],global_truss->obstacle.center);
      double p = global_truss->obstacle.partial(di);
      if (p != 0.0) {
	v += global_truss->obstacle.f(coords[j]);
      }
    }

   if (v < best_score) {
     if (debug)    cout << "found best: " << v << "\n";
     for (int i = 0; i < global_truss->var_edges; ++i) {
       best_distances[i] = ds(i);
     }
     best_score = v;
     
   }
   delete[] coords;
   return v;    
  }

  // compute the derivatives of the objective as the configuration ds.
  column_vector Invert::derivative(const column_vector& ds) {
    for (int i = 0; i < global_truss->var_edges; ++i) {
      if (debug) std::cout << i << " : " << ds(i) << std::endl;
      // This is correct?  It should it just be "i"?
      global_truss->distance(i+1) = ds(i);
    }
    // If I don't change an here, I'm not changing the coords!!
   column_vector *coords = new column_vector[global_truss->num_nodes];
    find_all_coords(global_truss,coords);
    column_vector d(global_truss->var_edges);


      
    for(int i = 0; i < global_truss->var_edges; i++) {
      // The true edge number is one higher than the index of the variable edges, since the first is fixed.
      int e = i + 1;
      column_vector dx(2);
      dx(0) = 0.0;
      dx(1) = 0.0;
      double prod = 0.0;
      for(int j = 0; j < global_truss->goals.size(); j++) {

	column_vector g = global_truss->goals[j];
	int idx = global_truss->goal_nodes[j];	
	column_vector c = coords[idx];
	
	column_vector d = global_truss->compute_goal_derivative_c(coords,e,global_truss->goal_nodes[j]);

	if ((d(0) > 1000.0) || (abs(d(1)) > 1000.0)) {
	  cout << "CRISIS!\n";
	  print_vec(d);
	  abort();
	}

	dx += (d * global_truss->goal_weights[j]);
	column_vector goal_direction = c - g;
	prod += dot(goal_direction,dx);
      }
      d(i) = prod;

      // Now for this edge, we will compute the contribution from obstacle inteference.
      // This can occur for any node which is after the edge in the kinematic chain.
      // The basic plan here is to run through all nodes and compute the
      // partial derivative of a change in distance. If there is no intersection,
      // this will be zero. If it is non-zero, we need to compute partial derivative
      // of the change in distance given the change in direction induced by the change
      // in the edge length. (There will technically be a recomputation of
      // something computed above which could be memoized, but I will not worry
      // about that now.

      double d_obst = 0.0;
      
      // first we will run through all nodes.
      int first_node = global_truss->large_node(e);
      // Note this creates a n^2 operation in the number of nodes---
      // this is ripe for optimization.
      for(int j = first_node; j < global_truss->num_nodes; j++) {
	// Now, does the partial derivative of this node exist?
	double di = distance_2d(coords[j],global_truss->obstacle.center);
	double p = global_truss->obstacle.partial(di);
	if (p != 0.0) {
	  column_vector nd = coords[j];
	  column_vector d = global_truss->compute_goal_derivative_c(coords,e,j);	  

	  column_vector n_to_center = global_truss->obstacle.center - nd;
	  double direction = dot(n_to_center,d);
	  d_obst +=  ( direction * p);
	}
      }
      if (d_obst != 0.0) {

	d(i) += d_obst;
      }
    }
    // cout << "DERIVATIVES" << "\n";
    // for(int i = 0; i < global_truss->var_edges; i++) {
    //   cout << "i " << i << " " << d(i) << "\n";
    // }
    delete[] coords;
    return d;
  }
  
