// playground.cpp -- Interactive tool for testing configuration of a robotic manipulator via gradient techniquies 
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
#include "playground3d.hpp"
#include <SDL.h>
#include <dlib/optimization.h>
#include <stdio.h>
#include <iostream>
#include <limits.h>
#include "Invert3d.hpp"
// #include "Obstacle.hpp"
#include "Tetrahelix.hpp"
#include <math.h>

// ToDos:
// *) Clean up the code
// *) Implement a circle render for drawing the obstacle
// *) Support multiple obstacles
// *) Create an API-level separation between the playground/rendered and the solver!

using namespace std;
using namespace dlib;


#define USE_DERIVATIVES 1

extern int debug;

// int debug = 1;

// Sadly, the way the objective and deriviate functions are called
// the do not allow general "client" data to be passed in, and they are static.
// So keeping a global variable is the only effective way to maintain information
// for solving the "forward" problem.
// Therefore "global_truss" is global.
Tetrahelix *Invert3d::global_truss = 0;

// Obstacle obstacle;

void solve_inverse_problem(Tetrahelix *an) {
  column_vector sp(an->var_edges);
  column_vector lb(an->var_edges);
  column_vector ub(an->var_edges);      

  // We need the Tetrahelix to have distances in order to iniitilize this meaningfully
  double upper_bound = 0;
  double lower_bound = -1;
  for (int i = 0; i < an->var_edges; i++) {

    int n = an->edge_number_of_nth_variable_edge(i);
    sp(i) = an->distance(n);
    
    lb(i) = an->lower_bound(i);
    ub(i) = an->upper_bound(i);
    if (ub(i) > upper_bound) upper_bound = ub(i);
    if ((lower_bound == -1) || (lb(i) < lower_bound)) lower_bound = lb(i);    
  }
  Invert3d inv;
  inv.an = an;
  inv.set_global_truss();
  //  inv.ob = ob;

  //  int n = an->var_edges;

    best_score = std::numeric_limits<float>::max();
    cerr << an->var_edges << "\n";
    cerr << an->num_edges << "\n";
    cerr << an->num_nodes << "\n";    
    for (int i = 0; i < an->var_edges; ++i) {
	   best_distances[i] = an->distance(i+3);
    }

    //    double score = 0.0;
    int n = an->var_edges;
    cout << "# var edges: " << n << "\n";

    cout << "goal node [solve_inverse] " << an->goals[0] << "\n";

    double score = find_min_box_constrained(
					    //bfgs_search_strategy(),
					    lbfgs_search_strategy(100),
					    //    cg_search_strategy(),
    			     //			     newton_search_strategy,
			    //    			     objective_delta_stop_strategy(1e-5),
    			     objective_delta_stop_strategy(1e-5),			    
    			     *Invert3d::objective,
    			     *Invert3d::derivative,
    			     sp,
    			     uniform_matrix<double>(n,1, lower_bound),  // lower bound constraint
    			     uniform_matrix<double>(n,1, upper_bound)   // upper bound constraint
    			     );
    // I uses this to get rid of the warning
    score = score + 0.0;
    cout << "got a score : " << score << "\n";
    for (int i = 0; i < an->var_edges; ++i) {
      an->distance(i+3) = best_distances[i];
    }
};


// ----------------------------------------------------------------------------------------

int mainx(Tetrahelix *an,column_vector* coords)
{
  cout << "goal node [mainx] " << an->goals[0] << "\n";  
    try
    {
      // However, that is a low priority until I get the playground working.
	{

	  for (int i = 0; i < an->num_edges; ++i) {
	    an->distance(i) = MEDIAN;
	  }

	  // WARNING: We are trying here to skip over fixed edge; this should really be computed!
	  for (int i = 1; i < an->num_edges; ++i) {
	    an->distance(i) = INITIAL;
	  }

	  //	  column_vector* coordsx = new column_vector[an->num_nodes];
	  cout << "goal node [mainx] " << an->goals[0] << "\n";
	  
	  solve_forward_find_coords(an,coords);
	  cout << "SOLVE_INVERSE\n";
	  solve_inverse_problem(an);

	  solve_forward_find_coords(an,coords);

	  //	  find_all_coords(an,coords);
	  // Invert3d inv;
	  // inv.an = an;
	  // inv.set_global_truss();
	  // column_vector sp(an->var_edges);
	  // for (int i = 0; i < an->var_edges; i++) {
	  //   sp(i) = an->distance(i + 1);
	  // }	  
	  // double final = inv(sp);
	  // std::cout << "inv(x) final : " << final << std::endl;
	  // for(int i = 0; i < an->num_nodes; i++) {
	  //   std::cout << " d["<< i << "]" << coords[i](0) << "," << coords[i](1) << std::endl;
	  // }
	  // should now deallocate coords

	}
    }
    catch (std::exception& e)
    {
        cout << e.what() << endl;
    }

    return 0;
}





// TODO: reorganized this so that initialization is in a separate routine

Tetrahelix *init_Tetrahelix() {

  // This needs to be generalized but for now I'm going to
  // just add an obstacle.  If I can successfully work around
  // an obstacle, that will be strong evidence that I have a valuable system.
  // obstacle.radius = 2.0;
  // obstacle.center = column_vector(2);
  // obstacle.center = -2.0, 10.0;
  // obstacle.weight = 4.0;


    Tetrahelix *an = new Tetrahelix(TRUSS_NODES,
		  UPPER_BOUND,
		  LOWER_BOUND,
		  MEDIAN,
		  INITIAL);

  int last_node = an->num_nodes - 1;
  double bx = ((last_node % 2) == 0) ? 0.0 : an->median_d * cos(30.0*M_PI/180);
  double by = (an->median_d/2.0) * last_node;
  // WARNING --- this is almost certainly wrong.
  double bz = 0.0;

  // These are examples of adding different goals...
  //  an->add_goal_node(an->num_nodes*1/3,1*bx/3+-2.0,1*by/3,0.5);
  //  an->add_goal_node(an->num_nodes*2/3,2*bx/3+ 5.0,2*by/3,0.5);  
  double mx = 1.0;
  double my = 1.0;
  double mz = 1.0;

  an->add_goal_node(an->num_nodes-1,bx+mx,by+my,bz+mz,1.0);
  int debug = 0;
  if (debug) {
    for(int i = 0; i < an->goal_nodes.size(); i++) {
      cout << "goal_nodes[" << i << "] " << an->goal_nodes[i] << "\n";
    }
  }

  column_vector coords[3];
  an->init_fixed_coords_to_z_axis_alignment(coords);
  an->init_fixed_coords(coords);
  return an;
}



void handle_goal_target_physical(Tetrahelix *an,  column_vector* coordsx,double x, double y, double z) {
  cout << "Interpreted as double!\n";
  column_vector gl(3);
  gl(0) = x;
  gl(1) = y;
  gl(2) = z;
  handle_goal_target(an,coordsx,gl);  
}

void handle_goal_target(Tetrahelix *an,  column_vector* coordsx,column_vector gl) {
  an->goals[an->goals.size() - 1] = gl;
  cout  << "goal : " << gl << "\n";
  cout << "goal node : " << an->goal_nodes[0] << "\n";
  best_score = std::numeric_limits<float>::max();	
  auto start = std::chrono::high_resolution_clock::now();
	
  mainx(an,coordsx);
	
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
  long long milliseconds = microseconds/ 1000.0;

  cout << "optimization tim: ms = " << milliseconds << "\n";
}

