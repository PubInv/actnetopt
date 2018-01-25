#include "Invert.h"

using namespace std;
using namespace dlib;

double* best_distances = 0;
double best_score;

Invert::Invert() {

}
  
  double Invert::operator() ( const column_vector& ds) const
  {
    cur_an = an;
    return objective(ds);
  }

  void  Invert::set_cur_an() {
    cur_an = an;
    if (best_distances == 0) {
     best_distances = new double[(cur_an->num_nodes-3)*2 +3];
    }
  }

  // This is currently computing the sum of the l2_norm of the goal point quandrances (square of distance).
  double Invert::objective(const column_vector& ds) {
    //    if (debug) std::cout << "CUR_AN " << cur_an << "\n";
    if (debug) std::cout << "OBJECTIVE INPUTS" << std::endl;
    for (int i = 0; i < cur_an->var_edges; ++i) {
           if (debug) std::cout << i << " : " << ds(i) << std::endl;
	  cur_an->distance(i+1) = ds(i);
    }

    //    if (debug) std::cout << "DISTANCES" << std::endl;
    
    column_vector *coords = new column_vector[cur_an->num_nodes];

    // If I don't change an here, I'm not changing the coords!!
    find_all_coords(cur_an,coords);
    for(int i = 0; i < cur_an->num_nodes; i++) {
      //       std::cout << " coords["<< i << "]" << coords[i](0) << "," << coords[i](1) << std::endl;
    }
    
    double v = 0.0;
    for(int i = 0; i < cur_an->goal_nodes.size(); i++) {
      int idx = cur_an->goal_nodes[i];
      if (debug) std::cout << "idx " <<  idx  << std::endl;      
      column_vector g = cur_an->goals[i];
      cout << "goal " << idx << "\n";
      print_vec(g);
      column_vector c = coords[idx];
      cout << "coords " << idx << "\n";
      print_vec(c);
      
      column_vector x(2);
      x(0) = g(0);
      x(1) = g(1);
      column_vector y(2);
      y(0) = c(0);
      y(1) = c(1);
      
      column_vector d(2);
      if (debug) std::cout << "Invert x " <<  x(0) <<  "," << x(1)  << std::endl;
      if (debug) std::cout << "Invert y " <<  y(0) <<  "," << y(1)  << std::endl;                  
      d = x - y;
      if (debug) std::cout << "Invert d " <<  d(0) <<  "," << d(1) <<  " " << std::endl;
      cout << " weight " << cur_an->goal_weights[i] << "\n";
      v += (cur_an->goal_weights[i]*distance_2d(x,y));
      cout << " distance " << distance_2d(x,y) << "\n";      
      cout << " v " << v << "\n";
      //      cout << "i v" << i << " " << v << "\n";
      //      v += l2_norm(d);
    }
   std::cout << "Invert v " <<  v <<  " " << std::endl;
   std::cout << "best_score " <<  best_score <<  " " << std::endl;   

   if (v < best_score) {
     cout << "found best: " << v << "\n";
    for (int i = 0; i < cur_an->var_edges; ++i) {
      best_distances[i] = ds(i);
    }
    best_score = v;
     
   }
   delete[] coords;
    return v;    
  }

  // compute the derivatives of the objective as the configuration ds.
  // NOTE: At present this only works with ONE goal node
  column_vector Invert::derivative(const column_vector& ds) {
    for (int i = 0; i < cur_an->var_edges; ++i) {
      if (debug) std::cout << i << " : " << ds(i) << std::endl;
      // This is correct?  It should it just be "i"?
      cur_an->distance(i+1) = ds(i);
    }
    // If I don't change an here, I'm not changing the coords!!
   column_vector *coords = new column_vector[cur_an->num_nodes];
    find_all_coords(cur_an,coords);
    column_vector d(cur_an->var_edges);


      
    for(int i = 0; i < cur_an->var_edges; i++) {
      // The true edge number is one higher than the index of the variable edges, since the first is fixed.
      int e = i + 1;
      column_vector dx(2);
      dx(0) = 0.0;
      dx(1) = 0.0;
      double prod = 0.0;
      for(int j = 0; j < cur_an->goals.size(); j++) {

	column_vector g = cur_an->goals[j];
	int idx = cur_an->goal_nodes[j];	
	column_vector c = coords[idx];
	
	column_vector d;
	//	cout << " e = " << e << "\n";
	if ((e % 2) == 1) {
	  d = cur_an->compute_external_effector_derivative_c(coords,e,cur_an->goal_nodes[j]);
	} else  if ((e % 2) == 0) {
	  d = cur_an->compute_internal_effector_derivative_c(coords,e,cur_an->goal_nodes[j]);
	}
	//	cout << "derivative  j d " << j << "\n";
	if ((d(0) > 1000.0) || (abs(d(1)) > 1000.0)) {
	  cout << "CRISIS!\n";
	  print_vec(d);
	  abort();
	}

	//	print_vec(d);
	//	print_vec(dx);
	//	cout << "weight " << cur_an->goal_weights[j] << "\n";
	dx += (d * cur_an->goal_weights[j]);
	column_vector goal_direction = c - g;
	prod += dot(goal_direction,dx);
      }
      d(i) = prod;
    }
    // cout << "DERIVATIVES" << "\n";
    // for(int i = 0; i < cur_an->var_edges; i++) {
    //   cout << "i " << i << " " << d(i) << "\n";
    // }
    delete[] coords;
    return d;
  }
  
