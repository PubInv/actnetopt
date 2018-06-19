// Invert.cpp -- Code needed to perform gradient-based inverse problem (design) of a 2D Warr
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

#include "Invert3d.hpp"
#include "Obstacle.hpp"

using namespace std;
using namespace dlib;


// These are globals, which is very unforatunate. However, dlib
// graditent routines do NOT reliably return the best value as the documentation says.
// Therefore these are necessary.
double* best_distances = 0;
double best_score;

int debug_inv = 0;

Invert3d::Invert3d() {

}
  
  double Invert3d::operator() ( const column_vector& ds) const
  {
    global_truss = an;
    return objective(ds);
  }

  void  Invert3d::set_global_truss() {
    global_truss = an;
    //        global_truss->obstacle = o;
    if (best_distances == 0) {
      // Is this really right?
      best_distances = new double[global_truss->num_edges];
    }
  }

  double Invert3d::objective(const column_vector& ds) {
    int debug = 0;
    if (debug) {
      cout << "OBJECTIVE:  \n"; 
    }

    //    if (debug_inv) std::cout << "OBJECTIVE INPUTS" << std::endl;
    for (int i = 0; i < global_truss->var_edges; ++i) {
      int n = global_truss->edge_number_of_nth_variable_edge(i);
      global_truss->distance(n) = ds(i);
      if (debug) {
	cout << ds(i) << " ";
      }
    }
      if (debug) {
	cout << "\n";
      }
    
  //   cout << "OBJECTIVE AAA\n";
  // for (int i = 0; i < global_truss->num_edges; ++i) {
  //   cout << " i, distances(i) " << i << " , " << global_truss->distance(i) << "\n";
  // }

  column_vector *coords = new column_vector[global_truss->num_nodes];

    // If I don't change an here, I'm not changing the coords!!
    solve_forward_find_coords(global_truss,coords);	  

    if (debug) {
    for(int i = 0; i < global_truss->num_nodes; i++) {
          std::cout << " coords["<< i << "]" << coords[i](0) << "," << coords[i](1) << "," << coords[i](2) << std::endl;
    }
    }
    
    double v = 0.0;
    for(int i = 0; i < global_truss->goal_nodes.size(); i++) {
      int idx = global_truss->goal_nodes[i];
      column_vector g = global_truss->goals[i];
      column_vector c = coords[idx];
      //      cout << "distance to goal : " << distance_3d(g,c) << "\n";
      v += (global_truss->goal_weights[i]*distance_3d(g,c));
    }

    //    cout << " v = " << v << "\n";
    
    // now we must add in the increase to the object caused by the obstacle!!
    // for(int j = 0; j < global_truss->num_nodes; j++) {
    //   // Now, does the partial derivative of this node exist?
    //   double di = distance_3d(coords[j],global_truss->obstacle.center);
    //   double p = global_truss->obstacle.partial(di);
    //   if (p != 0.0) {
    // 	v += global_truss->obstacle.f(coords[j]);
    //   }
    // }
    if (debug)
      cout << "v = " << v << "\n";
   if (v < best_score) {
     if (debug_inv)   cout << "found best: " << v << "\n";
     for (int i = 0; i < global_truss->var_edges; ++i) {
       best_distances[i] = ds(i);
       if (debug_inv) std::cout << i << " : " << ds(i) << std::endl;       
     }
     best_score = v;
   }
   delete[] coords;
   return v;    
  }

column_vector normalize3(column_vector v) {
  column_vector r(3);
  column_vector zero(3);
  zero(0) = 0.0;
  zero(1) = 0.0;
  zero(2) = 0.0;  
  double len = distance_3d(zero,v);
  r(0) = v(0)/len;
  r(1) = v(1)/len;
  r(2) = v(2)/len;  
  return r;
}

const bool USE_DIFFERENTIAL = false;
const bool USE_JACOBIAN = true;

  // compute the derivatives of the objective as the configuration ds.
column_vector Invert3d::derivative(const column_vector& ds) {
  int debug = 0;
  if (debug) {
  cout << "DERIVATIVE CALLED\n";
  }
  
  for (int i = 0; i < global_truss->var_edges; ++i) {
    int n = global_truss->edge_number_of_nth_variable_edge(i);
    global_truss->distance(n) = ds(i);
  }

  for (int i = 0; i < global_truss->num_edges; ++i) {
    if (debug) cout << " i, distances(i) " << i << " , " << global_truss->distance(i) << "\n";
  }

  // If I don't change an here, I'm not changing the coords!!
  column_vector *coords = new column_vector[global_truss->num_nodes];
  if (debug) cout << "NUM NODES " << global_truss->num_nodes << "\n";

  // This entire approach is very inefficent.
  // The derivatives are dependent on the coords; until we change the
  // coords we don't need to recompute these things. However, that will
  // require a major rework to make this obvious.
  solve_forward_find_coords(global_truss,coords);
  
  matrix<double> Ju = global_truss->Jacobian(coords,global_truss->num_nodes-1);
  cout << "Jacobian:\n";
  cout << Ju;
  cout << "End Jacobian\n";
  cout << "trying Ju\n";
  global_truss->Jacobian_temp = Ju;

  debug = 0;
  for (int i = 0; i < global_truss->num_nodes; ++i) {
    if (debug) cout << " nodes " << i <<  "\n";
    if (debug) print_vec(coords[i]);
  }
  debug = 0;
  
  column_vector d(global_truss->var_edges);
      
  for(int i = 0; i < global_truss->var_edges; i++) {
    // The true edge number is one higher than the index of the variable edges, since the first is fixed.
    int e = global_truss->edge_number_of_nth_variable_edge(i);
    column_vector dx(3);
    dx = 0.0,0.0,0.0;
    double prod = 0.0;
    for(int j = 0; j < global_truss->goals.size(); j++) {
      column_vector g = global_truss->goals[j];
      if (debug) {
	cout << "GOAL COORDS: \n";
	print_vec(g);
      }
      int idx = global_truss->goal_nodes[j];	
      column_vector c = coords[idx];

      if (debug) {
	cout << "GOAL INDEX: " << idx << "\n";
      }

      column_vector d;
      
      //      global_truss->compute_goal_derivative_j(coords,i,idx);
      
      if (USE_DIFFERENTIAL) {
	d = global_truss->compute_goal_differential_c(coords,e,idx);
      } else if (USE_JACOBIAN) {
	// There are a compute of problems here---I don't seem to really be using the goal_number here!!!
	// Note that this uses the variable number, NOT the true edge number.
	d = global_truss->compute_goal_derivative_j(coords,i,idx);
      } else {
	d = global_truss->compute_goal_derivative_c(coords,e,idx);	
      }

      if ((d(0) > 1000.0) || (abs(d(1)) > 1000.0)) {
	cout << "CRISIS!\n";
	print_vec(d);
	abort();
      }
      if (isnan(d(0))) {
	cout << "DERIVIATIVE NOT DEFINED! THIS MEANS WE ARE OPTIMAL!";
      }
      dx = (d * global_truss->goal_weights[j]);
      // This is the code I copied from Invert, but it seems wrong to me..
      // This is actually the anti-goal direction, but this is a way to
      // express that we want to move closer to the goal, so the derivative
      // goes UP as we move AWAY from the goal.
      column_vector goal_direction = c - g;

      if (l2_norm(goal_direction) == 0.0) { // In this case, the derivative is undefined...
	if (debug) cout << "goal_direction is null!\n";
	prod += 0.0;
      } else {

	double ds_de = dot(goal_direction,dx)/l2_norm(goal_direction);
	if (debug) {
	  cout << "ANALYSIS\n";
	  cout << "node position: ";
	  print_vec(c);

	  cout << "goal position: ";

	  print_vec(g);
      
	  cout << "goal direction: ";
	  print_vec(goal_direction);

	  cout << "derivative: ";
	  print_vec(d);
      
	  cout << "ds/de: ";
	  cout << ds_de;
	  cout << "\n";
	}

	prod += ds_de;
      }
      
    }
    //    cout << "edge,prod = " << e << " , " << prod <<"\n";    
    d(i) = prod;

    if (global_truss->obstacle != NULL) {

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
	double di = distance_3d(coords[j],global_truss->obstacle->center);
	double p = global_truss->obstacle->partial(di);
	//	cout << " presumably since we have no obstacles this should be zero!\n";
	//	cout << p << "\n";
	
	if (p != 0.0) {
	  column_vector deriv_v = global_truss->compute_goal_derivative_c(coords,e,j);	  

	  column_vector n_to_center = global_truss->obstacle->center - coords[j];

	  // What is "d" here!!! OMG!
	  double direction = dot(n_to_center,deriv_v);
	  d_obst +=  ( direction * p);
	}
      }

      if (d_obst != 0.0) {
	cout << "WHOA! Haven't defined an obstacle yet!\n";
	d(i) += d_obst;
      }
    }
  }
  debug = 0;
  if (debug)   cout << "DERIVATIVES" << "\n";
  for(int i = 0; i < global_truss->var_edges; i++) {
    int n = global_truss->edge_number_of_nth_variable_edge(i);
    if (debug) cout << "edge " << n << " " << d(i) << "\n";
    // WARNING: Experimental

  }
  delete[] coords;
  return d;
}
  
